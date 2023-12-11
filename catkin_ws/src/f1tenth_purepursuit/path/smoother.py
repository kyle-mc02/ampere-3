import pandas as pd
import sys
import matplotlib.pyplot as plt
import scipy as sci

df = pd.read_csv("%s.csv"%(sys.argv[1]), header=None)
print(df)

x, y = df.loc[:, 0], df.loc[:, 1]
#print(x)
#print(y)

xp, yp = sci.ndimage.gaussian_filter1d(x, 4), sci.ndimage.gaussian_filter1d(y, 4)

plt.plot(x, y)
plt.plot(xp, yp)
plt.show()

# Drop that column
df.drop(0, axis = 1, inplace = True)
df.drop(1, axis = 1, inplace = True)
df[0] = xp
df[1] = yp

df.to_csv("%s_smooth.csv"%(sys.argv[1]), header=False)