#!/usr/bin/env python3
"""
Stage installed rosidl .idl into the build tree under a single module name.

rosidl_generate_interfaces emits code whose C++ namespace and registration
symbols both follow the consuming project's name, so the module declared inside
each .idl must match it. The deployed Autoware/ROS .idl declare their own source
package as the module. This copies the INSTALLED .idl (content = the deployed
source of truth, never a committed copy) into the build tree and rewrites the
module / include / qualified-type references to a single namespace so they form
one consistent interface set. Only those names change; field order and types --
the wire contract -- are copied verbatim.

Staged files keep their {msg,srv}/Name.idl path (no package-name folder) so the
generators emit output under the package once; the staged paths are printed,
';'-joined, for the CMake interface tuples.

Usage: stage_idl.py <namespace> <out_dir> <pkg>=<share_dir>[ ...] -- <pkg/rel.idl> ...
"""
import os
import re
import sys


def main():
    argv = sys.argv[1:]
    sep = argv.index('--')
    namespace = argv[0]
    out_dir = argv[1]
    shares = dict(a.split('=', 1) for a in argv[2:sep])
    rels = argv[sep + 1:]

    pkgs = '|'.join(re.escape(p) for p in shares)
    re_include = re.compile(r'(#include\s+")(?:%s)(/(?:msg|srv)/)' % pkgs)
    re_module = re.compile(r'^(\s*module\s+)(?:%s)(\s*\{)' % pkgs, re.MULTILINE)
    re_type = re.compile(r'\b(?:%s)(::(?:msg|srv)::)' % pkgs)

    staged = []
    seen = {}
    for rel in rels:
        pkg = rel.split('/', 1)[0]
        sub = rel[len(pkg) + 1:]
        # Two packages with a same-named {msg,srv}/X.idl flatten to one staged
        # leaf; the second would silently overwrite the first (a message would
        # vanish from the codec). Refuse it.
        if sub in seen and seen[sub] != rel:
            raise SystemExit(
                'idl leaf collision: %s from %s and %s' % (sub, seen[sub], rel))
        seen[sub] = rel
        src = os.path.join(shares[pkg], sub)
        with open(src) as fh:
            text = fh.read()
        text = re_include.sub(r'\1%s\2' % namespace, text)
        text = re_module.sub(r'\1%s\2' % namespace, text)
        text = re_type.sub(r'%s\1' % namespace, text)
        # Stage at <out_dir>/{msg,srv}/X.idl (no package-name folder) so the
        # generators place output under <pkg>/{msg,srv}/ once, not doubled.
        dst = os.path.join(out_dir, sub)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        with open(dst, 'w') as fh:
            fh.write(text)
        staged.append(sub)

    sys.stdout.write(';'.join(staged))


if __name__ == '__main__':
    main()
