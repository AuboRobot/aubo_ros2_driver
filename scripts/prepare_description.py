#!/usr/bin/env python3
"""Remove unused dangling DAE links in the public aubo_description revision.

Keep the submodule pinned to its public commit. Never substitute another
robot's meshes, remove referenced assets, or overwrite valid links.
"""
import argparse
from pathlib import Path


def prepare(description):
    if not (description / 'package.xml').is_file():
        raise RuntimeError('Initialize aubo_description with git submodule update --init --recursive first')
    candidates = []
    references = [p.read_text(encoding='utf-8') for p in (description / 'urdf').rglob('*')
                  if p.is_file() and p.suffix in ('.urdf', '.xacro', '.srdf')]
    for index in range(6):
        relative = f'meshes/aubo_i10HL/visual/link{index}.DAE'
        link = description / relative
        if not link.is_symlink() or link.exists():
            continue
        expected = f'../../aubo_i10L/visual/link{index}.DAE'
        if str(link.readlink()) != expected:
            raise RuntimeError(f'Unexpected dangling link: {link}')
        if any(relative in text for text in references):
            raise RuntimeError(f'Refusing to remove referenced mesh: {relative}')
        candidates.append(link)
    for link in candidates:
        link.unlink()
        print(f'Removed unused dangling link: {link.relative_to(description)}')
    print(f'Description prepared: {len(candidates)} unused dangling links removed')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--description', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'aubo_description')
    prepare(parser.parse_args().description)
