import sys
def test_sp_probe():
    print("SYSPATH0:", sys.path[:4])
    import omniheyoka
    print("OMNI:", omniheyoka.__file__)
