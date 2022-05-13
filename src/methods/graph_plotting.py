def ouput_plot(abscissaList, ordinateList):
    """    
    Works
    """
    abscissaList = np.asarray(abscissaList)
    ordinateList = np.asarray(ordinateList)
    B_spline_coeff = make_interp_spline(abscissaList, ordinateList)
    X_Final = np.linspace(abscissaList.min(), abscissaList.max(),75) #choose the resolution as 5010
    Y_Final = B_spline_coeff(X_Final)

    plt.plot(X_Final, Y_Final)
    plt.ylabel('Distance to wall (m)') #set the label for y axis
    plt.xlabel('Time (s)') #set the label for x-axis
    plt.title("Wall proximity") #set the title of the graph
    plt.grid()
    plt.show() #display the graph

def save_to_csv(listToSave, csvName):
    """
    Works: but watch out with csv file name when running the code multiple times
    """
    name = csvName + ".csv" #"Saved data angular"+ ".csv" # change name according to recording type
    dict = {name: listToSave}
    df = pd.DataFrame(dict)
    df.to_csv(name)
    np.savetxt(name, listToSave, delimiter =", ", fmt ='% s')

def show_output_results():
    saveThis = []
    saveThis.append(abscissaList)
    saveThis.append(ordinateList)      
    saveThis.append(ordinateList2)   
    csvName = "Data plot"    
    save_to_csv(saveThis, csvName)

    # Plotting the data
    ouput_plot(abscissaList, ordinateList)
    ouput_plot(abscissaList, ordinateList2)
    return None
