from numpy import sqrt, cos, sin, pi
var = 0
speed_circle = sqrt(((-pi*sin(pi*var/25)) / (25))**2 + ((-pi*sin(pi*var/100)) / (20))**2 + ((pi*cos(pi*var/100)) / (20))**2)

print(speed_circle)



for i in range(100):
    var = i
    speed_torus = sqrt(((-pi*(80*cos(pi*var/200)*sin(pi*var/35)+14*sin(pi*var/200)*cos(pi*var/35)+49*sin(pi*var/200))) / (1400))**2 
    + ((-pi*(80*sin(pi*var/200)*sin(pi*var/35)-14*cos(pi*var/200)*cos(pi*var/35)-49*cos(pi*var/200))) / (1400))**2 
    + ((2*pi*cos(pi*var/35)) / (35))**2)

    print(speed_torus)

#df = pd.read_csv('')
#
#fart = [np.sqrt(df['u'][i]**2+df['v'][i]**2+df['w'][i]**2) for i in range(len(df['u']))]
#
#full_sec1 = full_sec(df)
#
#plt.plot(full_sec1, fart)
#
#print(max(fart))
#
#plt.show()