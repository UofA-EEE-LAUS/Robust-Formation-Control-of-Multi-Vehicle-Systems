
#Alpha trimmer filter, uses the stream to take values of "windowsize" and "alpha" to filter unusually large or small data from the stream.


def alpha_mean(window, alpha):
    '''
    Parameters
    ----------
    window : Int
        DESCRIPTION: Window size for alpha trimming 
    alpha : Int
        DESCRIPTION: Alpha size trimmer for alpha trimming

    Yields
    ------
    Float
        DESCRIPTION: filtered data output
    '''
    #Parsing mean data from the stream
    cut = (alpha//2)

    #Trimming the window, this should ideally be [cut:-cut] but the first two values of the list are highly error prone
    data = sorted(window)[cut+1:-cut] 
    store =[]
    for item in data:
        store.append(item)
    return sum(store)/len(data)

def alphatrimmer(window_size, alpha, sensor_stream):
    '''
    Parameters
    ----------
    window_size : Int
        DESCRIPTION: Window size for alpha trimming 
    alpha : Int
        DESCRIPTION: Alpha size trimmer for alpha trimming
    sensor_stream : Float
        DESCRIPTION: input noisy sensor stream 
    Yields
    ------
    Float
        DESCRIPTION: filtered data output
    '''
    window = []
    data_list = [] #empty list to store data
    for item in sensor_stream:
        window.append(item)
        if len(window) > window_size: #condition for filling the data to window size
            break
        x = ''.join(window)
        x = float(x)
        data_list.append(x)

    yield alpha_mean(data_list, alpha)

    for item in sensor_stream: #clearing windows and outputing filtered data
        window.pop(0)
        window.append(item)
        yield alpha_mean(window,alpha)
