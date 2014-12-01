total = 0
num = 0
while True:
    try:
        curr = int(raw_input())
    except:
        print float(total)/num
        exit()
    total += curr
    num += 1
