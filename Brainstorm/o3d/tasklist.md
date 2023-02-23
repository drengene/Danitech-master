# To-do list for LMAO project
<!-- Bullet list of things to do -->
The algorithm will compare images from the raycast and the real depth image to determine the differences. This will create an mask, that will tell how important each difference is. Factors will be
- Distance from the camera
  - Where further away objects are less important
- Edges of objects
  - Where edges are less important, because they are more likely to be wrong
- The normal of the object in the digital twin
  - As the vehicle will be driving on the surfaces, those with normals closer to the direction of gravity will be more important, and should thus have higher resolution.
  - Surfaces with normals opposite to the direction of gravity will be less important, and should thus have lower resolution.
- Magnitude of the difference
  - it is more important to update the model where the magnitude of the difference is larger