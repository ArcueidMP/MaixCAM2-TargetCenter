"""桩：sys.memory_info()"""


def memory_info():
    return {"cma_total": 0, "cma_used": 0, "cmm_total": 1073741824,
            "cmm_used": 0, "hw_total": 4294967296,
            "total": 1073741824, "used": 0}
