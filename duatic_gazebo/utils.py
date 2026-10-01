import os


def join_env_paths(*paths: str) -> str:
    """
    Join a list of env paths into an os.pathsep-separated string, filtering
    out empty strings and stripping leading/trailing os.pathsep characters.
    """
    return os.pathsep.join(filter(None, [path.strip(os.pathsep) for path in paths]))
