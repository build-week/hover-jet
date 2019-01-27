try:
    from colorama import init as colorama_init
    from colorama import Fore, Style
    colorama_init()

except(ImportError):
    print "Could not import colorama: Skipping coloration"

    class Dummy(object):
        def __getattr__(self, key):
            return ''

    Fore = Dummy()
    Style = Dummy()

from collections import OrderedDict


class Log(object):
    _verbosities = OrderedDict([
        ("debug", Fore.LIGHTBLUE_EX),
        ("info", Fore.WHITE),
        ("success", Fore.LIGHTGREEN_EX),
        ("warn", Fore.YELLOW),
        ("error", Fore.RED),
    ])
    _enabled = _verbosities.keys()

    @classmethod
    def set_enabled(cls, keys):
        for key in keys:
            assert key in cls._verbosities.keys(), "{} unknown".format(key)
        cls._enabled = keys

    @classmethod
    def set_verbosity(cls, level):
        assert level in cls._verbosities.keys(), "{} unknown".format(level)
        all_keys = cls._verbosities.keys()
        start = all_keys.index(level)
        cls.set_enabled(all_keys[start:])
        cls.debug("Enabling ", all_keys[start:])

    @classmethod
    def get_verbosities(cls):
        return cls._verbosities


def make_logger(verbosity_type, color):
    def new_logger(cls, *txt):
        if verbosity_type in cls._enabled:
            out_str = " ".join(map(str, txt))
            print("{}{}{}".format(color, out_str, Style.RESET_ALL))

    return new_logger


for verbosity, color in Log._verbosities.items():
    setattr(Log, verbosity, classmethod(make_logger(verbosity, color)))


if __name__ == '__main__':
    Log.error("error:", 0)
    Log.warn("warn:", 1)
    Log.success("success:", 2)
    Log.info("info:", 3)
    Log.debug("debug:", 4)
