#!/usr/bin/env python3
"""Fetch exact upstream commits into a local, reusable source cache."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parent.parent
DEPS = Path(os.environ.get('XOVIUM_DEPS_DIR', ROOT / '.build/deps')).resolve()
CATALOG = json.loads((ROOT / 'app/dependencies.json').read_text())
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--backend-only', action='store_true', help='Prepare only the JSON dependency for native command line tools')
args = parser.parse_args()
if args.backend_only:
    CATALOG = {'json': CATALOG['json']}
# Reuse only matching stamped sources. The editor remains independently buildable.
CACHE_ROOTS = [ROOT.parent / 'xovium-installer/.build/deps']
if os.environ.get('XOVIUM_SOURCE_CACHE'):
    CACHE_ROOTS.insert(0, Path(os.environ['XOVIUM_SOURCE_CACHE']).resolve())
DEPS.mkdir(parents=True, exist_ok=True)
for name, spec in CATALOG.items():
    destination = DEPS / name
    stamp = destination / '.xovium-commit'
    if stamp.is_file() and stamp.read_text().strip() == spec['commit']:
        continue
    print(f"Dependency: {name} {spec['tag']} ({spec['commit']})", flush=True)
    with tempfile.TemporaryDirectory(dir=DEPS, prefix=name + '-') as temporary:
        temporary = Path(temporary)
        extracted = temporary / 'extracted'
        cached = next((base / name for base in CACHE_ROOTS
                       if (base / name / '.xovium-commit').is_file()
                       and (base / name / '.xovium-commit').read_text().strip() == spec['commit']), None)
        if cached:
            shutil.copytree(cached, extracted, ignore=shutil.ignore_patterns('.git'))
        else:
            repository = temporary / 'git'
            subprocess.run(['git', 'init', '-q', str(repository)], check=True)
            subprocess.run(['git', '-C', str(repository), 'fetch', '--depth=1', spec['repository'], spec['commit']], check=True)
            actual = subprocess.check_output(['git', '-C', str(repository), 'rev-parse', 'FETCH_HEAD'], text=True).strip()
            if actual != spec['commit']:
                raise RuntimeError('Dependency commit mismatch: ' + name)
            archive = temporary / 'source.tar'
            with archive.open('wb') as stream:
                subprocess.run(['git', '-C', str(repository), 'archive', spec['commit']], stdout=stream, check=True)
            extracted.mkdir()
            with tarfile.open(archive) as stream:
                stream.extractall(extracted, filter='data')
        (extracted / '.xovium-commit').write_text(spec['commit'] + '\n')
        if destination.exists():
            shutil.rmtree(destination)
        extracted.rename(destination)
