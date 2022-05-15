import configparser
import os
import errno

def read_ini(config_ini_path, section):
    config_ini = configparser.ConfigParser()
    if not os.path.exists(config_ini_path):
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), config_ini_path)
    config_ini.read(config_ini_path, encoding="utf-8")
    read_section = config_ini[section]
    return read_section