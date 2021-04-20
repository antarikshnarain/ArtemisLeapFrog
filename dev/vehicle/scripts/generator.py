import os
from numpy import arange

def generator(file_datas):
    new_data = []
    size = len(file_datas)
    i = 0
    mrange = []
    delay = 0
    while i < size:
        # print(file_datas[i])
        if file_datas[i][0] == '#':
            var_name,user_range = file_datas[i].split(' ')
            start,stop,step = user_range.split(':')
            mrange = [round(val,4) for val in arange(float(start), float(stop), float(step))]
            i += 1
        else:
            cmd = file_datas[i]
            delay = file_datas[i+1]
            if len(mrange) > 0:
                for val in mrange:
                    new_data.append(cmd.replace(var_name, str(val)))
                    new_data.append(delay)
                mrange = []
            else:
                new_data.append(cmd)
                new_data.append(delay)
            i += 2
    return new_data
            

if __name__ == "__main__":
    files = os.listdir("raw/")
    for file in files:
        print("Generating for ", file, end=' ')
        new_file = open(file,'w')
        old_file = open("raw/" + file, 'r')
        new_data = generator(old_file.readlines())
        new_file.writelines(new_data)
        old_file.close()
        new_file.close()
        print("Done")
