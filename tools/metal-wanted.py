#!/usr/bin/env python3
"""metal-wanted.py LOG...: what the system asked of the Metal driver that it does not serve, from console logs
(bootrun.sh's test-console.log, usbmux-log.py): the driver's own lines -- refused pipelines and calls, selectors
answered by forwarding (zeros), things said once -- grouped by process and kind, most frequent first."""
import re, sys, collections
kinds = [('refused pipeline', r'refused: a (render|compute) pipeline: (.*)'),
         ('refused call', r'refused: (.*)'),
         ('unanswered selector', r'(.*(?:is declared by|answered with zeros|forward).*)'),
         ('not served', r'(.*(?:not served|is not done|stays non-volatile|was asked for).*)')]
seen = collections.defaultdict(collections.Counter)
for path in sys.argv[1:]:
    for line in open(path, errors='replace'):
        m = re.search(r'HakiosMetal\[(\S+) \d+\]: (.*)', line)
        if not m: continue
        who, text = m.group(1), m.group(2).strip()
        for kind, pattern in kinds:
            k = re.match(pattern, text)
            if k:
                what = k.group(k.lastindex)
                if kind == 'refused pipeline':           # one line a pipeline and reason, not one a variant
                    what = re.sub(r' for pixel format \d+', '', what); what = re.sub(r' \(fragment constants[^)]*\)', '', what)
                    what = k.group(1) + ': ' + what
                what = re.sub(r'IOSurface \d+', 'IOSurface N', what); what = re.sub(r' at 0x[0-9a-f]+ \(unslid', ' (unslid', what)
                what = re.sub(r'\(block \d+, instruction \d+, opcode \d+\) ?', '', what)
                seen[kind][(who, what[:170])] += 1
                break
for kind, _ in kinds:
    if not seen[kind]: continue
    print('%s: %d different' % (kind, len(seen[kind])))
    for (who, what), n in seen[kind].most_common(40): print('  %5d  %-14s %s' % (n, who, what))
