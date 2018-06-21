def send_command(cmd, travel_time):
    control_socket.send_multipart([b'motor',pickle.dumps(cmd,0)])

    time.sleep(travel_time)
    stop = (0.0,0.0) # stopping
    control_socket.send_multipart([b'motor',pickle.dumps(stop,0)])
