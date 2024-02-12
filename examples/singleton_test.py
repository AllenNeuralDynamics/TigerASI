from tigerasi.tiger_controller import TigerController

stage1 = TigerController(port='COM3')
stage2 = TigerController(port='COM3')

if id(stage1) == id(stage2):
    print("singleton works, both objects contain the same instance.")
else:
    print("singleton failed, objects contain different instances.")