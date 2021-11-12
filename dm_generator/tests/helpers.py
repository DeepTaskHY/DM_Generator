#!/usr/bin/python3
# -*- coding: utf-8 -*-
import glob
import os
from dtroslib.helpers import get_package_path
from pathlib import Path


def get_testbench_path() -> str:
    return os.path.join(get_package_path('dm_generator'), 'tests/msgs')


def get_msgs_path(testbench_name: str) -> str:
    return os.path.join(get_testbench_path(), testbench_name)


def get_dir_list(base_path: str):
    return (entry for entry in Path(base_path).iterdir() if entry.is_dir())


def get_testbench_list():
    return [entry.name for entry in get_dir_list(get_testbench_path())]


def get_msgs_list(testbench_name: str):
    file_list = sorted(glob.glob(os.path.join(get_msgs_path(testbench_name), '*')))
    msgs_list = sorted([file for file in file_list if file.endswith('.json')], key=len)

    return msgs_list
