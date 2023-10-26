import PySimpleGUI as sg
import serial as ser
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.ticker as mticker
import time
import matplotlib
import keyboard

def wait_for_ready_signal(s):
    # Implement your handshake protocol to wait for the ready signal
    # For example, read from the serial port until the ready signal is received
    ready_signal = b"READY\r\n"  # Customize this according to your protocol
    while True:
        data = s.readline()
        if data == ready_signal:
            break


def Object_Ditector(masking_distance):

    s = ser.Serial('COM1', baudrate=9600, bytesize=ser.EIGHTBITS,
                   parity=ser.PARITY_NONE, stopbits=ser.STOPBITS_ONE,
                   timeout=1)  # timeout of 1 sec so that the read and write operations are blocking,
    s.flush()  # clear the port
    enableTX = True
    s.set_buffer_size(1024, 1024)
    # clear buffers
    s.reset_input_buffer()
    s.reset_output_buffer()
    dist_map = [-1 for x in range(61)]
    deg_map = [-1 for x in range(61)]
    flag1 = 1
    flag2 = 1
    k = 0
    j = 0


    while (flag1 or flag2):
        while (s.in_waiting > 0):  # while the input buffer isn't empty
            while (flag1 == 1):
                if k == 61:
                    k = 0
                    break
                received_data = s.read()  # read  from the buffer until the terminator is received,

                dist_map[k] = int.from_bytes(received_data, byteorder='big')
                time.sleep(0.1)
                k += 1
                if dist_map[k - 1] == 0:
                    flag1 = 0
                    k = 0
            while (flag2 == 1):  # recive deg
                if k == 61:
                    k = 0
                    break
                received_data = s.read()  # read  from the buffer until the terminator is received,
                temp = int.from_bytes(received_data, byteorder='big')
                time.sleep(0.1)
                # readline() can also be used if the terminator is '\n'
                # res = int(received_data)
                # g = int.from_bytes(received_data,byteorder='big')
                deg_map[k] = temp
                k += 1
                if deg_map[k - 1] == 0:
                    flag2 = 0
                deg_map[k-1] = (temp - 1) * 3
            if flag2 == 0 and flag1 == 0:
                break

            if (s.in_waiting == 0):
                enableTX = True

        # TX

        while (s.out_waiting > 0 or enableTX):  # while the output buffer isn't empty

            i = 2

            bytetxState = bytes('1', 'ascii')
            s.write(bytetxState)
            time.sleep(0.1)  # delay for accurate read/write operations on both ends
            # while True:
            if s.out_waiting == 0 and i == 0:
                # bytetx_masking_distance = bytes('/0', 'ascii')
                # s.write(bytetx_masking_distance)
                time.sleep(0.1)
                s.reset_output_buffer()
                break
            while i > 0:
                base = chr(int(masking_distance) % 80 + 48)
                offset = chr(int(masking_distance) // 80 + 48)
                if i == 2:
                    bytetx_masking_distance = bytes(base, 'ascii')
                elif i == 1:
                    bytetx_masking_distance = bytes(offset, 'ascii')
                s.write(bytetx_masking_distance)
                time.sleep(0.1)
                s.reset_output_buffer()
                i -= 1

            time.sleep(0.1)  # delay for accurate read/write operations on both ends
            if s.out_waiting == 0:
                enableTX = False




    return dist_map, deg_map



def Light_Detector():
    s = ser.Serial('COM1', baudrate=9600, bytesize=ser.EIGHTBITS,
                   parity=ser.PARITY_NONE, stopbits=ser.STOPBITS_ONE,
                   timeout=1)  # timeout of 1 sec so that the read and write operations are blocking,
    # after the timeout the program continues
    s.flush()  # clear the port
    enableTX = True
    s.set_buffer_size(1024, 1024)
    # clear buffers
    s.reset_input_buffer()
    s.reset_output_buffer()
    volt_map = [-1 for x in range(120)]
    deg_map = [-1 for x in range(61)]
    flag1 = 1
    flag2 = 1
    flag3 = 1
    k = 0
    j = 0

    while (flag1 or flag2):

        while (s.in_waiting > 0):  # while the input buffer isn't empty
            while (flag1 == 1 or flag3 == 1):
                if k == 120:
                    k = 0
                    break
                received_data = s.read()  # read  from the buffer until the terminator is received,
                volt_map[k] = int.from_bytes(received_data, byteorder='big')
                time.sleep(0.1)
                k += 1
                if volt_map[k - 1] == 0 and flag1 ==0:
                    flag3 = 0
                    k = 0
                    break
                if volt_map[k - 1] == 0:
                    flag1 = 0
                    k += -1

            while (flag2 == 1):#recive deg
                if k == 61:
                    k = 0
                    break
                received_data = s.read()  # read  from the buffer until the terminator is received,
                temp = int.from_bytes(received_data, byteorder='big')
                time.sleep(0.1)
                # readline() can also be used if the terminator is '\n'
                # res = int(received_data)
                # g = int.from_bytes(received_data,byteorder='big')
                deg_map[k] = temp
                k += 1
                if deg_map[k - 1] == 0:
                    flag2 = 0
                deg_map[k-1] = (temp-1) * 3
            if flag2 == 0 and flag1 == 0:
                break

            if (s.in_waiting == 0):
                    enableTX = True

        # TX

        while (s.out_waiting > 0 or enableTX):  # while the output buffer isn't empty

            bytetxState = bytes('3', 'ascii')
            s.write(bytetxState)
            time.sleep(0.1)  # delay for accurate read/write operations on both ends
            while True:
                # bytetx_masking_distance = bytes('/0', 'ascii')
                # s.write(bytetx_masking_distance)
                time.sleep(0.1)
                s.reset_output_buffer()
                break

            if s.out_waiting == 0:
                enableTX = False
    s.close()
    return volt_map,deg_map


def calculate_LDR_Calibration_volt(LDR_Calibration):
    LDR_Calibration_volt = [0 for x in range(50)]
    mean_volt = LDR_Calibration
    j = 0
    while (j < 9):
        v0 = mean_volt[j]
        v1 = mean_volt[j+1]
        m = (v1 - v0) / 5
        k = v0 - (j * 5) * m
        y = 0
        while (y < 5):
            LDR_Calibration_volt[5 * j + y] = (5 * j + y) * m + k
            y += 1
        j += 1

    return LDR_Calibration_volt


def calculate_dis_Lights(volt_map, LDR_Calibration_volt ):
    index = volt_map.index(0)
    index = (index + 1)//2
    volt_lst = [((((volt_map[x] * 100) + (volt_map[index+ x]))/2 ) * 3.3) / 4095 for x in range(index)]
    volt_lst = volt_lst[:index] + [0 for x in range(60 - index)]
    dist_map = [0 for x in range(61)]
    delta_map = [abs(LDR_Calibration_volt[x + 6] - LDR_Calibration_volt[x + 5]) for x in range(43)]
    maximum = max(delta_map)
    j = 0
    while (j < 60):
        i = 0
        while i < 50:
            if volt_lst[j] == 3.3:
                i+=1
                continue
            if abs(volt_lst[j] - LDR_Calibration_volt[i]) < maximum:
                dist_map[j] = i
                break
            i += 1
        j += 1
    return dist_map


def Object_AND_Light_Detector():
    s = ser.Serial('COM1', baudrate=9600, bytesize=ser.EIGHTBITS,
                   parity=ser.PARITY_NONE, stopbits=ser.STOPBITS_ONE,
                   timeout=1)  # timeout of 1 sec so that the read and write operations are blocking,
    # after the timeout the program continues
    s.flush()  # clear the port
    enableTX = True
    s.set_buffer_size(1024, 1024)
    # clear buffers
    s.reset_input_buffer()
    s.reset_output_buffer()
    dist_map = [-1 for x in range(61)]
    deg_map  = [-1 for x in range(61)]
    deg_map1 = [-1 for x in range(61)]
    volt_map = [-1 for x in range(120)]
    deg_map2 = [-1 for x in range(61)]
    flag1 = 1
    flag2 = 1
    flag3 = 1
    flag4 = 1
    flag5 = 1
    k = 0
    j = 0

    while (flag1 or flag2):

        while (s.in_waiting > 0):  # while the input buffer isn't empty


            while (s.in_waiting > 0):  # while the input buffer isn't empty
                while (flag4 == 1):
                    received_data = s.read()  # read  from the buffer until the terminator is received,

                    dist_map[k] = int.from_bytes(received_data, byteorder='big')
                    time.sleep(0.1)
                    k += 1
                    if dist_map[k - 1] == 0:
                        flag4 = 0
                        k = 0
                while (flag5 == 1):  # recive deg
                    received_data = s.read()  # read  from the buffer until the terminator is received,
                    temp = int.from_bytes(received_data, byteorder='big')
                    time.sleep(0.1)
                    # readline() can also be used if the terminator is '\n'
                    # res = int(received_data)
                    # g = int.from_bytes(received_data,byteorder='big')
                    deg_map1[k] = temp
                    k += 1
                    if deg_map1[k - 1] == 0:
                        k = 0
                        flag5 = 0
                    deg_map1[k-1] = (temp-1) * 3




                while (flag1 == 1 or flag3 == 1):
                    received_data = s.read()  # read  from the buffer until the terminator is received,
                    volt_map[k] = int.from_bytes(received_data, byteorder='big')
                    time.sleep(0.1)
                    k += 1
                    if volt_map[k - 1] == 0 and flag1 == 0:
                        flag3 = 0
                        k = 0
                        break
                    if volt_map[k - 1] == 0:
                        flag1 = 0
                        k += -1

                while (flag2 == 1):  # recive deg
                    received_data = s.read()  # read  from the buffer until the terminator is received,
                    temp = int.from_bytes(received_data, byteorder='big')
                    time.sleep(0.1)
                    # readline() can also be used if the terminator is '\n'
                    # res = int(received_data)
                    # g = int.from_bytes(received_data,byteorder='big')
                    deg_map2[k] = temp
                    k += 1
                    if deg_map2[k - 1] == 0:
                        flag2 = 0
                    deg_map2[k - 1] = (temp - 1) * 3
                if flag2 == 0 and flag1 == 0:
                    break

                if (s.in_waiting == 0):
                    enableTX = True

        # TX

        while (s.out_waiting > 0 or enableTX):  # while the output buffer isn't empty

            bytetxState = bytes('4', 'ascii')
            s.write(bytetxState)
            time.sleep(0.1)  # delay for accurate read/write operations on both ends
            while True:
                # bytetx_masking_distance = bytes('/0', 'ascii')
                # s.write(bytetx_masking_distance)
                time.sleep(0.1)
                s.reset_output_buffer()
                break

            if s.out_waiting == 0:
                enableTX = False
    s.close()
    j = 0
    while j<60:
        if (deg_map1[j] == -1 or deg_map1[j] == 0) and (deg_map1[j] > 0):
            deg_map[j] = deg_map2[j]
            j += 1
        else:
            deg_map[j] = deg_map1[j]
            j += 1


    return volt_map, dist_map, deg_map


def map_objectdetector(dist_map,deg_map, partision, state):
    s = ser.Serial('COM1', baudrate=9600, bytesize=ser.EIGHTBITS,
                   parity=ser.PARITY_NONE, stopbits=ser.STOPBITS_ONE,
                   timeout=1)  # timeout of 1 sec so that the read and write operations are blocking,
    # after the timeout the program continues
    s.flush()  # clear the port
    s.set_buffer_size(1024, 1024)
    # clear buffers
    s.reset_input_buffer()
    s.reset_output_buffer()
    matplotlib.use('TKAgg')
    data = [0 for x in range(61)]
    if state == 1 or state == 2:
        color = 'red'
        label = 'Radar Object Detection'
        min_dis = 1
    elif state == 3:
        color = 'yellow'
        label = 'Light Source Detection'
        min_dis = 5

    # Convert the data to integers (except the last element)
    k = 0

    if(state == 1 or state == 3):
        while(deg_map[k] != 0):
            data[deg_map[k]-1] = dist_map[k]
            k += 1
            if k == 61:
                k = 0
                break
    elif(state == 2):
        data = dist_map
    data = [int(x) if isinstance(x, int) else 0 for x in data[:-1]]
    angles = np.arange(0, 181, 1)
    theta = angles * (np.pi / 180)
    fig = plt.figure(facecolor='k')
    fig.canvas.toolbar.pack_forget()
    fig.canvas.manager.set_window_title('Ultrasonic Radar')
    mgn = plt.get_current_fig_manager()
    mgn.window.state('zoomed')
    r_max = max(dist_map) + 10
    ax = fig.add_subplot(1, 1, 1, polar=True, facecolor='#006b70')
    ax.tick_params(axis='both', colors='w')
    ax.set_ylim([0.0, r_max])  # Adjust the radial axis limits if needed
    ax.set_xlim([0.0, np.pi])  # Adjust the radial axis limits if needed
    ax.set_position([-0.05, -0.05, 1.1, 1.05])
    ax.set_rticks(np.linspace(0.0, r_max, 5))
    ax.set_thetagrids(np.linspace(0.0, 180, 10))

    angles = np.arange(0, 180, partision)
    theta = angles * (np.pi / 180)
    distances = data[:len(angles)]
    pols, = ax.plot(np.radians(angles),data, linestyle='', marker='o', markerfacecolor=color, markeredgecolor='w', markeredgewidth=0.3,
                    markersize=3.0,
                    alpha=0.5)
    line1 = ax.plot(np.radians(angles),distances, color='w', linewidth=0.3)

    axbackground = fig.canvas.copy_from_bbox(ax.bbox)
    enableTX = True
    bytetxState = bytes('0', 'ascii')
    s.write(bytetxState)
    time.sleep(0.1)  # delay for accurate read/write operations on both ends
    s.close()


    return




def map_objectdetector_state4(dist_map,volt_map,deg_map):
    s = ser.Serial('COM1', baudrate=9600, bytesize=ser.EIGHTBITS,
                   parity=ser.PARITY_NONE, stopbits=ser.STOPBITS_ONE,
                   timeout=1)  # timeout of 1 sec so that the read and write operations are blocking,
    # after the timeout the program continues
    s.flush()  # clear the port
    s.set_buffer_size(1024, 1024)
    # clear buffers
    s.reset_input_buffer()
    s.reset_output_buffer()
    matplotlib.use('TKAgg')
    angles = np.arange(0, 181, 1)
    theta = angles * (np.pi / 180)
    dists = np.ones((181,))

    # Convert the data to integers (except the last element)
    k = 0

    fig = plt.figure(facecolor='k')
    fig.canvas.toolbar.pack_forget()
    fig.canvas.manager.set_window_title('Ultrasonic Radar & Light Detector')
    mgn = plt.get_current_fig_manager()
    mgn.window.state('zoomed')

    ax = fig.add_subplot(111, polar=True, facecolor='#006b70')
    ax.tick_params(axis='both', colors='w')
    ax.set_ylim([0.0, 50])  # Adjust the radial axis limits if needed
    ax.set_xlim([0.0, np.pi])  # Adjust the radial axis limits if needed
    ax.set_position([-0.05, -0.05, 1.1, 1.05])
    ax.set_rticks(np.linspace(0.0, 50, 5))
    ax.set_thetagrids(np.linspace(0.0, 180, 10))


    axbackground = fig.canvas.copy_from_bbox(ax.bbox)

    line1 = ax.plot([], color='w', linewidth=0.1)
    ax.xaxis.set_major_locator(mticker.MaxNLocator(8))
    ticks_loc = ax.get_xticks().tolist()
    ax.xaxis.set_major_locator(mticker.FixedLocator(ticks_loc))
    fig.canvas.draw()
    axbackground = fig.canvas.copy_from_bbox(ax.bbox)

    t = 0
    while(t<60):
        if dist_map[t] > 0 and volt_map[t] <=0:
            dist = dist_map[t]
            angles = deg_map[t]
            color = 'r'
            t += 1
        else:
            dist = volt_map[t]
            angles = deg_map[t]
            color = 'y'
            t += 1

        pols, = ax.plot(np.radians(angles), dists[int(angles)], linestyle='', marker='o', markerfacecolor=color,
                        markeredgecolor='w', markeredgewidth=0.3,
                        markersize=3.0,
                        alpha=0.5)

        pols.set_data(np.repeat((angles * (np.pi / 180)), 2),
                      np.linspace(0.0, dist, 2))
        ax.draw_artist(pols)
        fig.canvas.blit(ax.bbox)
        fig.canvas.flush_events()
        time.sleep(0.2)
    bytetxState = bytes('0', 'ascii')
    s.write(bytetxState)
    time.sleep(0.1)  # delay for accurate read/write operations on both ends
    s.close()


    return


def Telemeter(degree_value,R_max):
    matplotlib.use('TKAgg')
    s = ser.Serial('COM1', baudrate=9600, bytesize=ser.EIGHTBITS,
                   parity=ser.PARITY_NONE, stopbits=ser.STOPBITS_ONE,
                   timeout=2)  # timeout of 1 sec so that the read and write operations are blocking,

    r_max = int(R_max)
    s.flush()  # clear the port
    enableTX = True
    s.set_buffer_size(1024, 1024)
    # clear buffers
    s.reset_input_buffer()
    s.reset_output_buffer()
    flag = 0
    angles = np.arange(0, 181, 1)
    theta = angles * (np.pi / 180)
    counter = 0
    dists = np.ones((181,))
    temp = 0
    while True:

        if flag == 0 and enableTX == False:
            break
        while flag:
            try:
                data = s.read()
                decoded = int.from_bytes(data, byteorder='big')
                vals = float(decoded)*2 + float(decoded)%2
                dist = vals
                angles = float(degree_value)
                counter += 1


                if dist < r_max and abs(dist - temp) > 1:
                    dists[int(angles)] = dist
                    temp = dist

                elif dist > r_max :
                    dists[int(angles)] = 0.0
                else:
                    dists[int(angles)] = temp
                    dist = temp
                if counter == 1:
                    fig = plt.figure(facecolor='k')
                    fig.canvas.toolbar.pack_forget()
                    fig.canvas.manager.set_window_title('Ultrasonic Radar You Need to Long Press q In Keyboard To Exit')
                    mgn = plt.get_current_fig_manager()
                    mgn.window.state('zoomed')

                    ax = fig.add_subplot(111, polar=True, facecolor='#006b70')
                    ax.tick_params(axis='both', colors='w')
                    ax.set_ylim([0.0, r_max])  # Adjust the radial axis limits if needed
                    ax.set_xlim([0.0, np.pi])  # Adjust the radial axis limits if needed
                    ax.set_position([-0.05, -0.05, 1.1, 1.05])
                    ax.set_rticks(np.linspace(0.0, r_max, 5))
                    ax.set_thetagrids(np.linspace(0.0, 180, 10))

                    pols, = ax.plot(np.radians(angles), dists[int(angles)], linestyle='', marker='o', markerfacecolor='r',
                                    markeredgecolor='w', markeredgewidth=0.3,
                                    markersize=3.0,
                                    alpha=0.5)


                    axbackground = fig.canvas.copy_from_bbox(ax.bbox)

                    line1 = ax.plot([], color='w', linewidth=0.1)
                    ax.xaxis.set_major_locator(mticker.MaxNLocator(8))
                    ticks_loc = ax.get_xticks().tolist()
                    ax.xaxis.set_major_locator(mticker.FixedLocator(ticks_loc))
                    fig.canvas.draw()
                    axbackground = fig.canvas.copy_from_bbox(ax.bbox)

                ax.draw_artist(pols)
                pols.set_data(np.repeat((angles * (np.pi / 180)), 2),
                               np.linspace(0.0, dist, 2))
                fig.canvas.blit(ax.bbox)
                fig.canvas.flush_events()
                time.sleep(2)
                if keyboard.is_pressed('q'):
                    flag = 0
                    bytetxState = bytes('0', 'ascii')
                    s.write(bytetxState)
                    time.sleep(0.25)
                    break


            except KeyboardInterrupt:
                flag = 0
                plt.close('all')
                bytetxState = bytes('0', 'ascii')
                s.write(bytetxState)
                time.sleep(0.25)  # delay for accurate read/write operations on both ends
                # while True:
                print('keyboard interrupt')
                break

            figure_numbers = plt.get_fignums()

            if len(figure_numbers) == 0:
                bytetxState = bytes('0', 'ascii')
                s.write(bytetxState)
                time.sleep(0.1)  # delay for accurate read/write operations on both ends
                flag = 0
                continue
        while ((s.out_waiting > 0 or enableTX) and flag == 0):  # while the output buffer isn't empty

            i = 1

            bytetxState = bytes('2', 'ascii')
            s.write(bytetxState)
            time.sleep(0.25)  # delay for accurate read/write operations on both ends
            # while True:
            if s.out_waiting == 0 and i == 0:
                time.sleep(0.25)
                s.reset_output_buffer()
                break
            while i > 0:
                base = int(degree_value)
                if i == 1:
                    hex_string = chr(base)
                    bytetx_masking_distance = bytes(hex_string, 'ascii')

                s.write(bytetx_masking_distance)
                time.sleep(0.25)
                s.reset_output_buffer()
                i -= 1

            time.sleep(0.25)  # delay for accurate read/write operations on both ends
            if s.out_waiting == 0:
                enableTX = False
                flag = 1
                break

    bytetxState = bytes('0', 'ascii')
    s.write(bytetxState)
    time.sleep(0.1)  # delay for accurate read/write operations on both ends
    s.close()
    return




def BOOT_CAMP(x):
    s = ser.Serial('COM1', baudrate=9600, bytesize=ser.EIGHTBITS,
                   parity=ser.PARITY_NONE, stopbits=ser.STOPBITS_ONE,
                   timeout=1)  # timeout of 1 sec so that the read and write operations are blocking,

    bytetxState = bytes('6', 'ascii')
    if x == 'y':
        bytetxCalibartion = bytes('y', 'ascii')
    elif x == 'n':
        bytetxCalibartion = bytes('n', 'ascii')

    LDR_Calibration_MSB = [0 for x in range(10)]
    LDR_Calibration_LSB = [0 for x in range(10)]
    temp = [0 for x in range(40)]
    t = 0
    k = 0
    m = 0
    j = 0
    e = 0
    s.flush()  # clear the port
    enableTX = True
    s.set_buffer_size(1024, 1024)
    # clear buffers
    s.reset_input_buffer()
    s.reset_output_buffer()
    i = 2
    while (t < 20):

        while (s.in_waiting > 0):  # while the input buffer isn't empty
            received_data = s.read()  # read  from the buffer until the terminator is received,
            temp[t] = int.from_bytes(received_data, byteorder='big')
            time.sleep(0.25)
            t += 1
            if t == 20:
                break

            if (s.in_waiting == 0):
                enableTX = True

        # TX

        while (i > 0):
            while (s.out_waiting > 0 or enableTX):  # while the output buffer isn't empty


                # while True:
                if s.out_waiting == 0 and i == 1:
                    s.write(bytetxCalibartion)
                    time.sleep(0.1)
                    s.reset_output_buffer()
                    i += -1
                    break
                elif s.out_waiting == 0 and i == 2:
                    s.write(bytetxState)
                    time.sleep(0.25)  # delay for accurate read/write operations on both ends
                    i += -1
                    enableTX = True
                while True:
                    # bytetx_masking_distance = bytes('/0', 'ascii')
                    # s.write(bytetx_masking_distance)
                    time.sleep(0.25)
                    break

            if s.out_waiting == 0:
                enableTX = False

    while k < 10:
        LDR_Calibration_MSB[k] = temp[k]
        k += 1
    while m < 10:
        LDR_Calibration_LSB[m] = temp[k + m]
        m += 1

    LDR_Calibration = [LDR_Calibration_MSB[x] * 50 + (LDR_Calibration_LSB[x]/2) for x in range(10)]
    LDR_Calibration_volt = [LDR_Calibration[x] * 3.3 / 4095 for x in range(10)]

    s.close()

    return LDR_Calibration_volt


def translator_to_hex(input_file, output_file):
    command_mapping = {
        "inc_lcd": "01",
        "dec_lcd": "02",
        "rra_lcd": "03",
        "set_delay": "04",
        "clear_lcd": "05",
        "servo_deg": "06",
        "servo_scan": "07",
        "sleep": "08"
    }

    with open(input_file, 'r') as input_fp:
        lines = input_fp.readlines()

    hex_commands = []

    for line in lines:
        line = line.strip()
        if not line:
            continue

        parts = line.split()  # return parts = [opcode, num] by whitespace
        command = parts[0]
        hex_value = command_mapping.get(command)
        if hex_value:
            if len(parts) > 1:
                args = parts[1].split(',')  # return parts = [opcode, num] by comma
                for arg in args:
                    hex_value += format(int(arg), '02X')
            hex_commands.append(hex_value)

    with open(output_file, 'w') as output_fp:
        for hex_value in hex_commands:
            output_fp.write(hex_value + '\n')


def Send_File(s, output_file):
    with open(output_file, 'r') as output_fp:
        code_hex = output_fp.readlines()

    for line in code_hex:
        j = 0
        while (line):
            if (line[j] == '\n'):
                Send_Char(s, ',')
                break
            Send_Char(s, line[j])
            j += 1


def Send_Char(s, chr):
    enableTX = True
    bytetxState = bytes(chr, 'ascii')
    s.write(bytetxState)
    time.sleep(0.01)  # delay for accurate read/write operations on both ends

def Send_Char_Without_S(chr):
    s = ser.Serial('COM1', baudrate=9600, bytesize=ser.EIGHTBITS,
                   parity=ser.PARITY_NONE, stopbits=ser.STOPBITS_ONE,
                   timeout=1)  # timeout of 1 sec where the read and write operations are blocking,
    Send_Char(s, '5')
    bytetxState = bytes(chr, 'ascii')
    s.write(bytetxState)
    time.sleep(0.1)  # delay for accurate read/write operations on both ends
    s.close()




def List_Hex_Files():
    input_file_1 = "D:\\Users\klainern\Desktop\Scripts\Script1_code.txt"
    output_file_1 = "D:\\Users\klainern\Desktop\Scripts\Script1.txt"

    input_file_2 = "D:\\Users\klainern\Desktop\Scripts\Script2_code.txt"
    output_file_2 = "D:\\Users\klainern\Desktop\Scripts\Script2.txt"

    input_file_3 = "D:\\Users\klainern\Desktop\Scripts\Script3_code.txt"
    output_file_3 = "D:\\Users\klainern\Desktop\Scripts\Script3.txt"
    lst_input_file = [input_file_1, input_file_2, input_file_3]
    lst_output_file = [output_file_1, output_file_2, output_file_3]

    for i in range(3):
        translator_to_hex(lst_input_file[i], lst_output_file[i])
    return lst_output_file


def Script_to_Buffer(num_file, file):
    s = ser.Serial('COM1', baudrate=9600, bytesize=ser.EIGHTBITS,
                   parity=ser.PARITY_NONE, stopbits=ser.STOPBITS_ONE,
                   timeout=1)  # timeout of 1 sec where the read and write operations are blocking,
    # after the timeout the program continues
    enableTX = False
    # clear buffers
    s.reset_input_buffer()
    s.reset_output_buffer()

    Send_Char(s, '5')
    #print("send 5")

    if num_file == 1:
        Send_Char(s, 'X')
        #print("send X")
        Send_File(s, file)
        #print("send File A")
        Send_Char(s, 'x')
        #print("send x mean write File A to Mem")


    if num_file == 2:
        Send_Char(s, 'Y')
        #print("send Y")
        Send_File(s, file)
        #print("send File B")
        Send_Char(s, 'y')
        print("send y mean write File B to Mem")


    if num_file == 3:
        Send_Char(s, 'Z')
        #print("send Z")
        Send_File(s, file)
        #print("send File C")
        Send_Char(s, 'z')
        #print("send z mean write File C to Mem")

    if num_file == 4:
        Send_Char(s, 'G')
        #print("send G")
        #print("Send Delete Mem Flash")

    s.close()

def GUI_Load_Start_File(File,num,char,window):

    user_input_layout = [
        [sg.Text("Please Select your Choice", justification='center', size=(50, 1), font='courier 15',
                 background_color="white")],
        [sg.Yes("Start File", size=(20, 1), font='Courier 15'), sg.No("Load File", size=(20, 1), font='Courier 15')]
    ]
    user_input_window = sg.Window("", user_input_layout)
    while True:
        user_event, user_values = user_input_window.read()
        if user_event == sg.WINDOW_CLOSED:
            break
        elif user_event == "Start File":
            Send_Char_Without_S(char)
            user_input_window.close()
            window.close()
            break
        elif user_event == "Load File":
            Script_to_Buffer(num, File)
            user_input_window.close()
            window.close()
            break



def GUI():
    LDR_Calibration_volt = [-1 for x in range(50)]
    while True:

        sg.theme('LightBlue2')
        layout = [
            [sg.Text("DCS Project - Roee Shamoon & Noam Klainer", size=(50, 1), justification='center',
                     font='Courier 15', background_color="white")],
            [sg.Button("1) Objects Detector System", size=(50, 1), font='Courier 15')],
            [sg.Button("2) Telemeter", size=(50, 1), font='Courier 15')],
            [sg.Button("3) Light Sources Detector System", size=(50, 1), font='Courier 15')],
            [sg.Button("4) Light Sources and Objects Detector System", size=(50, 1), font='Courier 15')],
            [sg.Button("5) Script Mode", size=(50, 1), font='Courier 15')],
            [sg.Button("6) Calibration", size=(50, 1), font='Courier 15')],
            [sg.Button("Exit", size=(50, 1), font='Courier 15')]
        ]


        window = sg.Window(title="Light Source & Object Proximity Detector System", element_justification='c',
                           layout=layout, size=(700, 350))

        event, values = window.read()


        if event == "Exit" or event == sg.WIN_CLOSED:
            break
        elif event == "1) Objects Detector System":
            plt.close('all')
            user_input_layout = [
                [sg.Text("Enter Masking Distance [CM]:", font='Courier 13')],
                [sg.Input()],
                [sg.Button("OK")]
            ]
            user_input_window = sg.Window("Masking Distance", user_input_layout)
            while True:
                user_event, user_values = user_input_window.read()

                if user_event == sg.WINDOW_CLOSED or user_event == "OK":
                    masking_distance = user_values[0]
                    break

            user_input_window.close()
            window.close()
            dist_map,deg_map = Object_Ditector(masking_distance)
            state1 = 1
            map_objectdetector(dist_map,deg_map, 3, state1)
            continue

        elif event == "2) Telemeter":

            plt.close('all')
            user_input_layout = [
                [sg.Text("Enter Degree of Telemeter [°]: ", font='Courier 13')],
                [sg.Input()],
                [sg.Text("Enter Rmax for scan of Telemeter [cm]: ", font='Courier 13')],
                [sg.Input()],
                [sg.Button("OK")]
            ]
            user_input_window = sg.Window("Degree Value", user_input_layout)
            while True:
                user_event, user_values = user_input_window.read()

                if user_event == sg.WINDOW_CLOSED or user_event == "OK":
                    degree_value = user_values[0]
                    R_max = user_values[1]
                    break

            user_input_window.close()
            window.close()


            Telemeter(degree_value,R_max)



            # s.write(2)

        elif event == "3) Light Sources Detector System":

            plt.close('all')
            window.close()
            volt_map,deg_map = Light_Detector()
            dist_light_map = calculate_dis_Lights(volt_map,LDR_Calibration_volt)
            state3 = 3
            map_objectdetector(dist_light_map,deg_map,3,state3)


            continue

        elif event == "4) Light Sources and Objects Detector System":
            window.close()
            plt.close('all')
            volt_map,dis_map, deg_map = Object_AND_Light_Detector()
            dist_light_map = calculate_dis_Lights(volt_map, LDR_Calibration_volt)
            map_objectdetector_state4(dist_light_map,dis_map, deg_map)




        elif event == "5) Script Mode":
            plt.close('all')
            lst_Files = List_Hex_Files()
            # print("Send F to MCU")
            user_input_layout = [
                [sg.Text("please select a file ", font='Courier 13', size=(50, 1), justification='center',
                         background_color="white")],
                [sg.Button("File 1", font='Courier 13', size=(50, 1))],
                [sg.Button("File 2", font='Courier 13', size=(50, 1))],
                [sg.Button("File 3", font='Courier 13', size=(50, 1))],
                [sg.Button("Delete Files", font='Courier 13', size=(50, 1))],
            ]
            user_input_window = sg.Window("", user_input_layout)
            while True:
                user_event, user_values = user_input_window.read()

                if user_event == sg.WIN_CLOSED:
                    break
                elif user_event == "File 1":
                    GUI_Load_Start_File(lst_Files[0], 1, 'i', user_input_window)
                    user_input_window.close()
                    window.close()
                    break
                elif user_event == "File 2":
                    GUI_Load_Start_File(lst_Files[1], 2, 'j', user_input_window)
                    user_input_window.close()
                    window.close()
                    break
                elif user_event == "File 3":
                    GUI_Load_Start_File(lst_Files[2], 3, 'k', user_input_window)
                    user_input_window.close()
                    window.close()
                    break
                elif user_event == "Delete Files":
                    #print("Delete Files")
                    Script_to_Buffer(4, lst_Files[0])
                    user_input_window.close()
                    window.close()
                    break
        elif event == "6) Calibration":

            user_input_layout = [
                [sg.Text("Do you need a calibration?", justification='center', size=(50, 1), font='courier 15',
                         background_color="white")],
                [sg.Yes("Yes", size=(20, 1), font='Courier 15'), sg.No("No", size=(20, 1), font='Courier 15')]
            ]
            user_input_window = sg.Window("", user_input_layout)
            while True:
                user_event, user_values = user_input_window.read()
                if user_event == sg.WINDOW_CLOSED:
                    break
                elif user_event == "Yes":
                    user_input_window.close()
                    window.close()
                    LDR_Calibration = BOOT_CAMP('y')
                    LDR_Calibration_volt = calculate_LDR_Calibration_volt(LDR_Calibration)
                    break
                elif user_event == "No":
                    user_input_window.close()
                    window.close()
                    LDR_Calibration = BOOT_CAMP('n')
                    LDR_Calibration_volt = calculate_LDR_Calibration_volt(LDR_Calibration)
                    break



def main():
    GUI()


if __name__ == '__main__':
    main()
