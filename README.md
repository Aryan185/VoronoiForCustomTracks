# VoronoiForCustomTracks
A Voronoi diagram is a simple concept, and it's based on the minimal distance needed to reach a landmark. If you need to go to a metro station, the most natural algorithm is going to the nearest one.

It forms circles of same size originating from the points that make up the track. The intersection of these circles are then joined together to finally form a continuous path connecting the starting and the ending point.
In this case, the points provided are actually the coordinates of cones laid 3 meters apart and represent a closed loop.

![Figure_1](https://user-images.githubusercontent.com/82220795/184881091-39ff911b-47ab-4754-aaec-e6a0bab72314.png)

Here, a general track layout is displayed where the black dots represent the obstacles or the cones.

# Results
After running the algorithm on this layout, we get the optimized path represented by the red line. This path is chosen by traversing through all the voronoi points and finding the shortest path connecting the starting and the ending points.

![Figure_1](https://user-images.githubusercontent.com/82220795/184882215-457a1fca-314a-4f00-9068-f13c902453c5.png)
