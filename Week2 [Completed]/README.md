# Repository RE-608 [WEEK-2 Assignment]
### STATUS = COMPLETED
---

Di assignment ini saya ditugaskan untuk membuat sebuah visualisasi pergerakan kaki 3 Joint (3 engsel) yang menggunakan logika gerak Forward Kinematic lalu mevisualisasikannya menggunakan MatPlotLib. <br>
Disini saya menggunakan referensi Inverse Kinematic Matplotlib dari teman saya yang dimana referensi ini saya gunakan untuk membuat versi Froward Kinematicnya, hasil dari program visualisasi saya 
dapat dilihat pada GIF dibawah ini: <br>

<br>
  <img src="./FK_Circular.gif" width="500" alt="Forward Kinematik Lingkaran"> <br>
<br>

Dari GIF diatas ujung kaki robot (Tibia End-point) menggambar atau tracing sebuah lingkaran yang dimana pergerakannya ini sepenuhnya menggunakan rumus Forward Kinematic. <br>
<br>
Rumus Forward Kinematic yang saya gunakan: <br>
```
a1 = θ1
P1 = origin + L_coxa  · [cos(a1), sin(a1)]

a2 = a1 + θ2
P2 = P1     + L_femur · [cos(a2), sin(a2)]

a3 = a2 + θ3
P3 = P2     + L_tibia · [cos(a3), sin(a3)]

aₙ = aₙ₋₁ + θₙ                        (mengkalkulasikan absolute angle)
Pₙ = Pₙ₋₁ + Lₙ · [cos(aₙ), sin(aₙ)]   (arah extensi)
```
