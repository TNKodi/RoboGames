
maze=[
    ["00", "01", "02", "03", "04", "05", "06", "07", "08", "09"],
    ["10", "11", "12", "13", "14", "15", "16", "17", "18", "19"],
    ["20", "21", "22", "23", "24", "25", "26", "27", "28", "29"],
    ["30", "31", "32", "33", "34", "35", "36", "37", "38", "39"],
    ["40", "41", "42", "43", "44", "45", "46", "47", "48", "49"],
    ["50", "51", "52", "53", "54", "55", "56", "57", "58", "59"],
    ["60", "61", "62", "63", "64", "65", "66", "67", "68", "69"],
    ["70", "71", "72", "73", "74", "75", "76", "77", "78", "79"],
    ["80", "81", "82", "83", "84", "85", "86", "87", "88", "89"],
    ["90", "91", "92", "93", "94", "95", "96", "97", "98", "99"]
]

red_yellow=[+1,-1,-1,+1,+1,-1,+1,+1,-1]
yellow_pink=[+1,-1,-1,+1,-1,-1,+1,+1,-1,-1]
pink_brown=[]
brown_green=[]




color_array=["red","yellow","pink","brown","green"]
color_path_array=[red_yellow,yellow_pink,pink_brown,brown_green]
color_path_index=0

current_path=color_path_array[color_path_index]
curr_index=0

def forward():
    pass
def stop():
    pass
def turnLeft():
    pass
def turnRight():
    pass

def wallfallowPID():
    pass


def isWalldetect():
    ps0=1
    ps7=1
    if(ps0>10 & ps7>10):
        stop()
        return True
    return False
def colourDetect():
    color="RED"
    return color



while(True):
    if(colourDetect()==color_array[color_path_index+1]):
        color_path_index+=1
        current_path=color_path_array[color_path_index]
        curr_index=0

    forward()
    while(~(isWalldetect())):
        wallfallowPID()
    #code for get color using camera

    if (current_path[curr_index]==1):
        turnLeft()
        stop()
        curr_index+=1
    elif(current_path[curr_index]==-1):
        turnRight()
        stop()
        curr_index+=1
        



