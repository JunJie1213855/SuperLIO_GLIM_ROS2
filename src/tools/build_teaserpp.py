#!/usr/bin/env python3
"""Build the pinned TEASER++ library and pinned transitive sources locally."""
import argparse
import subprocess
from pathlib import Path

DEPENDENCIES = {
    'pmc': ('https://github.com/jingnanshi/pmc.git', 'a2dfd612a501bca83c47206255dbbff619481f97'),
    'tinyply': ('https://github.com/ddiakopoulos/tinyply.git', '4495bb5757346f60f9910b8fcf848c878a8d54aa'),
    'spectra': ('https://github.com/jingnanshi/spectra.git', '5c4fb1de050847988faaaaa50f60e7d3d5f16143'),
}
TEASER_COMMIT = '52a9c52ee7d4c838c5e8a75458c33178be5bfb70'


def output(*cmd):
    return subprocess.check_output(cmd, text=True).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jobs', type=int, default=1)
    parser.add_argument('--offline', action='store_true', help='Use existing pinned checkouts only')
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error('--jobs must be positive')
    root = Path(__file__).resolve().parents[2]
    source = root / 'src/SuperLIO_SLAM/third_party/TEASER-plusplus'
    if output('git', '-C', str(source), 'rev-parse', 'HEAD') != TEASER_COMMIT:
        raise RuntimeError('Unexpected TEASER++ version; initialize the pinned submodule')
    if output('git', '-C', str(source), 'status', '--porcelain'):
        raise RuntimeError('TEASER++ source has local changes; preserve them before building pinned dependencies')
    vendor = root / 'build/teaserpp_vendor'
    vendor.mkdir(parents=True, exist_ok=True)
    flags = []
    for name, (url, commit) in DEPENDENCIES.items():
        path = vendor / name
        if not path.exists():
            if args.offline:
                raise RuntimeError(f'Missing offline dependency: {path}')
            subprocess.run(['git', 'clone', '--no-checkout', url, str(path)], check=True)
            subprocess.run(['git', '-C', str(path), 'checkout', '--detach', commit], check=True)
        if output('git', '-C', str(path), 'rev-parse', 'HEAD') != commit:
            raise RuntimeError(f'Unexpected dependency version: {path}')
        if output('git', '-C', str(path), 'status', '--porcelain'):
            raise RuntimeError(f'Dependency has local changes: {path}')
        flags.append(f'-DFETCHCONTENT_SOURCE_DIR_{name.upper()}={path}')
    build = root / 'build/teaserpp'
    subprocess.run(['cmake', '-S', str(source), '-B', str(build),
                    '-DCMAKE_BUILD_TYPE=Release', f'-DCMAKE_INSTALL_PREFIX={root / "install/teaserpp"}',
                    '-DBUILD_TESTING=OFF', '-DBUILD_DOC=OFF', '-DBUILD_PYTHON_BINDINGS=OFF',
                    '-DBUILD_MATLAB_BINDINGS=OFF', '-DBUILD_TEASER_FPFH=OFF',
                    '-DENABLE_DIAGNOSTIC_PRINT=OFF', *flags], check=True)
    subprocess.run(['cmake', '--build', str(build), '--parallel', str(args.jobs)], check=True)
    subprocess.run(['cmake', '--install', str(build)], check=True)


if __name__ == '__main__':
    main()
