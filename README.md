# AA2: Inverse Kinematics

## Team Description : Group B

Abel Pujol

abel.pujol@enti.cat

<img src='https://i.gyazo.com/e1f66a9470f2cb6892b108a7cba8d240.png' width='400'>



Adrià Pérez

adria.perez.garrofe@enti.cat

<img src='https://cdn.discordapp.com/attachments/913391373762326570/1166447312797716652/PXL_20230425_234301451.jpg?ex=654a85b6&is=653810b6&hm=42cc130ec6263409da57a351eac28d2c6b3ebf534eb2179afec0d1f7005627f3&' width='400'>


## Comments:

Our main source of issues and frustration came from the scene setup having inconsistencies or errors: The scorpion's left legs are flipped and the octopus stores references to the root gameobject of each tentacle instead of to the first bones. Additionally, the bone hierarchy is different between the scorpion and octopus. Since these issues are outside of the part of the project that we can modify, the solutions we have reached are not optimal.

That being said, all four exercises have been implemented to a functional level. The remaining issues we have been unable to fix are the scorpion legs flickering after the animation is over (could be related to the aforementioned setup issues) and the front tentacles twisting unnaturally, which could be fixed by improving our ccd implementation.
