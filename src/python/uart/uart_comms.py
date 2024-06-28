


### Loads the 

def readConfigFile(configFileName):
    """
    Reads the chirp configuration file.

    Parameters
    ----------
    file_path : string
        Path to the chrip config file.

    Returns
    -------
    list[string]
        Every command from the chirp config loaded into an array.
    """
    
    
    try:
        with open(configFileName, 'r') as fp:
            cnt = 0
            commands = []
            for line in fp:
                if (len(line) > 1):
                    if (line[0] != '%'):
                        commands.append(line)
                        cnt += 1
        return (cnt,commands)
    except FileNotFoundError as err:
        print("Chirp config file not found.")
    finally:
        fp.close()