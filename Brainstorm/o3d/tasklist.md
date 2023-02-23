# To-do list for LMAO project
<!-- Bullet list of things to do -->
## Attention
The algorithm will compare images from the raycast and the real depth image to determine the differences. This will create an mask, that will tell how important each difference is. Factors will be
- Distance from the camera
  - Where further away objects are less important
- Edges of objects
  - Where edges are less important, because they are more likely to be wrong
  - Maybe rate of change instead of binary edge.
- The normal of the object in the digital twin
  - As the vehicle will be driving on the surfaces, those with normals closer to the direction of gravity will be more important, and should thus have higher resolution.
  - Surfaces with normals opposite to the direction of gravity will be less important, and should thus have lower resolution.
- Magnitude of the difference
  - it is more important to update the model where the magnitude of the difference is larger

## Hypothesis
- Consider creating a hypothesis along the lines of
### Mapping
- By examining the features and the differences between the model and the real world, we ncan limit the amount of updates to the model, and thus reduce the amount of time it takes to update the model. This will allow us to update the model in real time. The model will also be less likely to be erroneous around difficult features, as they are only updated when the data is reliable.

### Localization
- By using the same algorithm, we can determine a subset of rays to cast for monte carlo localization


