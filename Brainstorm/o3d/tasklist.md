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


- Opdel billedet i zoner med ens gradienter. Ud fra denne kan de vigtigste trekanter udvælges. Hvis afstandsforskellen mellem gradientzone til nabozone er stor, skal en, fra kameraets synspunkt, usynlig trekant indsættes, således at næste zone kan indsættes med dybde.
- Derefter skal muligvis laves noget, som gør at de fundne zoner kan representeres af linjer, så de nemmere kan indsættes i modellen som trekanter.

- Den gradient jeg har skal konverteres til 3d space. Så undgår vi at den spheriske natur spiller os et puds.
- Vi vil gerne bare lappe bagsiden af de overflader vi ser med convex hull, for at undgå at lave surrfaces uden solid fundament.

## Hypothesis
- Consider creating a hypothesis along the lines of
### Mapping
- By examining the features and the differences between the model and the real world, we ncan limit the amount of updates to the model, and thus reduce the amount of time it takes to update the model. This will allow us to update the model in real time. The model will also be less likely to be erroneous around difficult features, as they are only updated when the data is reliable.

### Localization
- By using the same algorithm, we can determine a subset of rays to cast for monte carlo localization


