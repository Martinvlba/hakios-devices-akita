/*
 * The Pixel 8a's modem firmware, read where Android keeps it: the partition modem_a (or _b) holds an ext4 file system
 * with images/default/modem.bin in it (105 MB; "default" is a symbolic link to the build's directory). The file is the
 * vendor's and is never in this repository or in an image of ours: it is read from the phone's own storage, read-only
 * (drivers/storage/hakios.ufs.c takes no write for this partition at all).
 *
 * This is the little of ext4 that takes: the superblock, the group descriptors, inodes, directories searched block by
 * block (a hashed directory keeps its entries in ordinary directory blocks too), extent trees of any depth, and
 * symbolic links short enough to sit in the inode. No journal is replayed (the partition is mounted read-only by
 * Android as well), nothing is cached but the file's extents.
 *
 * The layout is ext4's on-disk format (Documentation/filesystems/ext4 of Linux: structures, offsets, magic numbers --
 * facts of the format; no code of Linux is here). Tried on the computer against a copy of the partition
 * (-DAKITA_MODEM_FS_TEST builds a program that reads PATH out of IMAGE).
 */
#ifdef AKITA_MODEM_FS_TEST
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define fs_alloc(n)     malloc(n)
#define fs_free(p, n)   free(p)
#define strlcpy(d, s, n) snprintf((d), (n), "%s", (s))  /* (glibc has none) */
#else
#include <kern/kalloc.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <sys/systm.h>
#include <sys/errno.h>
#define fs_alloc(n)     kalloc_data((n), Z_WAITOK | Z_ZERO)
#define fs_free(p, n)   kfree_data((p), (n))
#endif

#define EXT4_MAGIC              0xef53u
#define EXT4_EXTENT_MAGIC       0xf30au
#define EXT4_ROOT_INODE         2u
#define EXT4_INCOMPAT_64BIT     0x80u
#define EXT4_FLAG_EXTENTS       0x00080000u
#define EXT4_FLAG_INLINE        0x10000000u
#define EXT4_MODE_MASK          0xf000u
#define EXT4_MODE_DIR           0x4000u
#define EXT4_MODE_FILE          0x8000u
#define EXT4_MODE_LINK          0xa000u
#define MODEM_FS_MAX_EXTENTS    1024u

typedef int (*akita_modem_fs_read_t)(void *context, uint64_t offset, uint8_t *dst, uint64_t len);

struct akita_modem_fs_extent {
	uint64_t file_block;    /* the file's block this run begins at */
	uint64_t disk_block;    /* ... and where it is in the partition */
	uint32_t blocks;
};

struct akita_modem_file {
	akita_modem_fs_read_t read;
	void *context;
	uint32_t block_size;
	uint64_t size;
	uint32_t extent_count;
	struct akita_modem_fs_extent extents[MODEM_FS_MAX_EXTENTS];
};

struct ext4 {
	akita_modem_fs_read_t read;
	void *context;
	uint32_t block_size, inodes_per_group, inode_size, desc_size, first_data_block;
	uint64_t blocks;
};

static uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | p[1] << 8); }
static uint32_t le32(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }

static int
ext4_mount(struct ext4 *fs, akita_modem_fs_read_t read, void *context)
{
	uint8_t sb[1024];
	int error = read(context, 1024, sb, sizeof(sb));

	if (error != 0) {
		return error;
	}
	if (le16(sb + 56) != EXT4_MAGIC || le32(sb + 24) > 6) {
		return -1;
	}
	fs->read = read;
	fs->context = context;
	fs->block_size = 1024u << le32(sb + 24);
	fs->first_data_block = le32(sb + 20);
	fs->inodes_per_group = le32(sb + 40);
	fs->inode_size = le32(sb + 76) >= 1 ? le16(sb + 88) : 128;      /* (revision 0 has no field for it) */
	fs->desc_size = (le32(sb + 96) & EXT4_INCOMPAT_64BIT) != 0 && le16(sb + 254) >= 64 ? le16(sb + 254) : 32;
	fs->blocks = le32(sb + 4) | ((le32(sb + 96) & EXT4_INCOMPAT_64BIT) != 0 ? (uint64_t)le32(sb + 0x150) << 32 : 0);
	if (fs->inodes_per_group == 0 || fs->inode_size < 128 || fs->inode_size > fs->block_size) {
		return -1;
	}
	return 0;
}

/* inode number -> its first 160 bytes (the mode, the size, the flags, i_block) */
static int
ext4_inode(struct ext4 *fs, uint32_t number, uint8_t inode[160])
{
	uint32_t group = (number - 1) / fs->inodes_per_group, index = (number - 1) % fs->inodes_per_group;
	uint8_t desc[64];
	uint64_t table;
	int error;

	if (number == 0) {
		return -1;
	}
	memset(desc, 0, sizeof(desc));
	error = fs->read(fs->context, (uint64_t)(fs->first_data_block + 1) * fs->block_size + (uint64_t)group * fs->desc_size, desc,
	    fs->desc_size > 64 ? 64 : fs->desc_size);
	if (error != 0) {
		return error;
	}
	table = le32(desc + 8) | (fs->desc_size >= 64 ? (uint64_t)le32(desc + 0x28) << 32 : 0);
	if (table == 0 || table >= fs->blocks) {
		return -1;
	}
	memset(inode, 0, 160);
	return fs->read(fs->context, table * fs->block_size + (uint64_t)index * fs->inode_size, inode, fs->inode_size < 160 ? fs->inode_size : 160);
}

/* The extents under one node of the tree (the 60 bytes in the inode, or a block), in the file's order. */
static int
ext4_extents(struct ext4 *fs, const uint8_t *node, uint32_t node_bytes, struct akita_modem_file *file, unsigned level)
{
	uint32_t entries = le16(node + 2), depth = le16(node + 6);

	if (le16(node) != EXT4_EXTENT_MAGIC || 12 + entries * 12 > node_bytes || level > 5) {
		return -1;
	}
	for (uint32_t i = 0; i < entries; i++) {
		const uint8_t *e = node + 12 + i * 12;

		if (depth == 0) {
			uint32_t blocks = le16(e + 4);

			if (blocks > 32768) {           /* allocated, never written: reads as zeros; no firmware has one */
				return -1;
			}
			if (file->extent_count == MODEM_FS_MAX_EXTENTS) {
				return -1;
			}
			file->extents[file->extent_count].file_block = le32(e);
			file->extents[file->extent_count].disk_block = (uint64_t)le16(e + 6) << 32 | le32(e + 8);
			file->extents[file->extent_count].blocks = blocks;
			if (file->extents[file->extent_count].disk_block + blocks > fs->blocks) {
				return -1;
			}
			file->extent_count++;
		} else {
			uint64_t child = (uint64_t)le16(e + 8) << 32 | le32(e + 4);
			uint8_t *block;
			int error;

			if (child >= fs->blocks || (block = fs_alloc(fs->block_size)) == NULL) {
				return -1;
			}
			error = fs->read(fs->context, child * fs->block_size, block, fs->block_size);
			if (error == 0) {
				error = ext4_extents(fs, block, fs->block_size, file, level + 1);
			}
			fs_free(block, fs->block_size);
			if (error != 0) {
				return error;
			}
		}
	}
	return 0;
}

static int
ext4_file(struct ext4 *fs, const uint8_t inode[160], struct akita_modem_file *file)
{
	file->read = fs->read;
	file->context = fs->context;
	file->block_size = fs->block_size;
	file->size = le32(inode + 4) | (uint64_t)le32(inode + 0x6c) << 32;
	file->extent_count = 0;
	if ((le32(inode + 0x20) & EXT4_FLAG_EXTENTS) == 0 || (le32(inode + 0x20) & EXT4_FLAG_INLINE) != 0) {
		return -1;                              /* block maps and inline data: not what mke2fs makes here */
	}
	return ext4_extents(fs, inode + 0x28, 60, file, 0);
}

/*
 * Bytes [offset, offset + len) of an open file. A hole reads as zeros. 0, or what the partition's read answered.
 */
int akita_modem_file_read(struct akita_modem_file *file, uint64_t offset, uint8_t *dst, uint64_t len);
int akita_modem_fs_open(akita_modem_fs_read_t read, void *context, const char *path, struct akita_modem_file *file);
uint64_t akita_modem_file_size(const struct akita_modem_file *file);
size_t akita_modem_file_bytes(void);

int
akita_modem_file_read(struct akita_modem_file *file, uint64_t offset, uint8_t *dst, uint64_t len)
{
	if (offset > file->size || len > file->size - offset) {
		return -1;
	}
	while (len != 0) {
		uint64_t block = offset / file->block_size, within = offset % file->block_size, chunk = file->block_size - within;
		const struct akita_modem_fs_extent *found = NULL;
		int error;

		for (uint32_t i = 0; i < file->extent_count; i++) {
			if (block >= file->extents[i].file_block && block < file->extents[i].file_block + file->extents[i].blocks) {
				found = &file->extents[i];
				break;
			}
		}
		if (found != NULL) {            /* as much of the run as is asked for, in one read */
			chunk = (found->file_block + found->blocks - block) * file->block_size - within;
		}
		if (chunk > len) {
			chunk = len;
		}
		if (found == NULL) {
			memset(dst, 0, chunk);
		} else if ((error = file->read(file->context, (found->disk_block + (block - found->file_block)) * file->block_size + within,
		    dst, chunk)) != 0) {
			return error;
		}
		offset += chunk;
		dst += chunk;
		len -= chunk;
	}
	return 0;
}

/* One name in a directory: its inode number, 0 if it is not there. */
static uint32_t
ext4_lookup(struct ext4 *fs, const uint8_t dir_inode[160], const char *name, size_t name_len, struct akita_modem_file *scratch)
{
	uint8_t *block;
	uint32_t found = 0;

	if (ext4_file(fs, dir_inode, scratch) != 0 || (block = fs_alloc(fs->block_size)) == NULL) {
		return 0;
	}
	for (uint64_t at = 0; found == 0 && at + fs->block_size <= scratch->size; at += fs->block_size) {
		if (akita_modem_file_read(scratch, at, block, fs->block_size) != 0) {
			break;
		}
		for (uint32_t o = 0; o + 8 <= fs->block_size;) {
			uint32_t inode = le32(block + o), record = le16(block + o + 4), length = block[o + 6];

			if (record < 8 || o + record > fs->block_size) {
				break;
			}
			if (inode != 0 && length == name_len && o + 8 + length <= fs->block_size && memcmp(block + o + 8, name, name_len) == 0) {
				found = inode;
				break;
			}
			o += record;
		}
	}
	fs_free(block, fs->block_size);
	return found;
}

/*
 * Opens PATH ("images/default/modem.bin": names from the root, a symbolic link among them followed if its target is a
 * name in the same directory or a path from there, as Android's "default" is). 0 and the file; ENOENT-like -1 else.
 * The file structure is big (the extents): the caller allocates it.
 */
int
akita_modem_fs_open(akita_modem_fs_read_t read, void *context, const char *path, struct akita_modem_file *file)
{
	struct ext4 fs;
	uint8_t inode[160], parent[160];
	char rest[256];
	unsigned links = 0;

	if (ext4_mount(&fs, read, context) != 0 || ext4_inode(&fs, EXT4_ROOT_INODE, inode) != 0 || strlen(path) >= sizeof(rest)) {
		return -1;
	}
	strlcpy(rest, path, sizeof(rest));
	for (char *p = rest; *p != 0;) {
		char *end = p;
		size_t length;
		uint32_t number;

		while (*end != 0 && *end != '/') {
			end++;
		}
		length = (size_t)(end - p);
		if (length == 0) {              /* "a//b", a leading "/" */
			p = *end != 0 ? end + 1 : end;
			continue;
		}
		if ((le16(inode) & EXT4_MODE_MASK) != EXT4_MODE_DIR || (number = ext4_lookup(&fs, inode, p, length, file)) == 0) {
			return -1;
		}
		memcpy(parent, inode, sizeof(parent));
		if (ext4_inode(&fs, number, inode) != 0) {
			return -1;
		}
		p = *end != 0 ? end + 1 : end;
		if ((le16(inode) & EXT4_MODE_MASK) == EXT4_MODE_LINK) {
			/* a short link's target is in i_block itself; it replaces the name, the rest of the path follows it */
			uint32_t target = le32(inode + 4);
			char joined[256];

			if (target == 0 || target >= 60 || (le32(inode + 0x20) & EXT4_FLAG_EXTENTS) != 0 || inode[0x28] == '/' || ++links > 4 ||
			    snprintf(joined, sizeof(joined), "%.*s%s%s", (int)target, (const char *)inode + 0x28, *p != 0 ? "/" : "", p) >= (int)sizeof(joined)) {
				return -1;
			}
			strlcpy(rest, joined, sizeof(rest));
			p = rest;
			memcpy(inode, parent, sizeof(inode));
		}
	}
	if ((le16(inode) & EXT4_MODE_MASK) != EXT4_MODE_FILE) {
		return -1;
	}
	return ext4_file(&fs, inode, file);
}

uint64_t
akita_modem_file_size(const struct akita_modem_file *file)
{
	return file->size;
}

/* what a file structure takes (the extents make it big: allocate it) */
size_t
akita_modem_file_bytes(void)
{
	return sizeof(struct akita_modem_file);
}

#ifdef AKITA_MODEM_FS_TEST
static int
image_read(void *context, uint64_t offset, uint8_t *dst, uint64_t len)
{
	return fseek(context, (long)offset, SEEK_SET) != 0 || fread(dst, 1, len, context) != len;
}

int
main(int argc, char **argv)
{
	FILE *image = argc > 2 ? fopen(argv[1], "rb") : NULL;
	struct akita_modem_file *file = malloc(sizeof(*file));
	uint8_t *data;

	if (image == NULL || akita_modem_fs_open(image_read, image, argv[2], file) != 0) {
		fprintf(stderr, "not found\n");
		return 1;
	}
	fprintf(stderr, "%llu bytes in %u extents\n", (unsigned long long)file->size, file->extent_count);
	data = malloc(file->size);
	if (akita_modem_file_read(file, 0, data, file->size) != 0) {
		return 1;
	}
	fwrite(data, 1, file->size, stdout);
	return 0;
}
#endif
