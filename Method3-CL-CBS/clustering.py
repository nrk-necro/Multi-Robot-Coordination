import matplotlib.pyplot as plt
import numpy as np
from sklearn.cluster import AgglomerativeClustering
from sklearn.datasets import make_blobs
from sklearn.preprocessing import StandardScaler
from sklearn import datasets

X = np.unique(np.random.randint(10, size=(20,2)),axis=0)
n=2
clustering = AgglomerativeClustering(n_clusters=n).fit(X)
labels = clustering.labels_
d={0:[],1:[]}


print(labels)

# Number of clusters in labels, ignoring noise if present.
n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)

# Plot result

# Black removed and is used for noise instead.
unique_labels = set(labels)
print(unique_labels)
colors = ['y', 'b', 'g', 'r']

for i in range(len(X)):
    d[labels[i]].append(X[i])
    col=colors[labels[i]]
    plt.plot(X[i][0], X[i][1], 'o', markerfacecolor=col,
    		markeredgecolor='k',
    		markersize=6)

print(len(d[0]),len(d[1]))
    
plt.title('number of clusters: %d' % n_clusters_)
plt.show()
