import os
from dtroslib.helpers import get_package_path


def create_directory(dir_path: str):
    os.makedirs(os.path.dirname(dir_path), exist_ok=True)


def get_dataset_path(dataset_path: str, require_directory=False) -> str:
    path = os.path.join(get_package_path('dm_generator'), 'datasets', dataset_path)

    if require_directory:
        create_directory(path)

    return path


def get_model_path(model_path: str, require_directory=False) -> str:
    path = os.path.join(get_package_path('dm_generator'), 'models', model_path)

    if require_directory:
        create_directory(path)

    return path
