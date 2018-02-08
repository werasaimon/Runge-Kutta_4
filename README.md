# Runge-Kutta-4-Integration: IDE_QT

Introduction

This module integrates a system of ordinary differential equations of the form

![begineqnarray-yt-ft-yt-yt_0-y_0-endeqnarray-0298eae3db](https://user-images.githubusercontent.com/10780778/35971972-a25a0336-0cd8-11e8-8047-4795255e70c4.png)

where y is a vector of length n. Given time step \Delta t, the Runge-Kutta 4 method integrates the with update


![begineqnarray-y_n1-fracdelta-t6leftk_1-2k_2-2-41157480a7](https://user-images.githubusercontent.com/10780778/35972069-05d61ecc-0cd9-11e8-8825-6cedb519dfa3.png)

where ![k_n-d413726dee](https://user-images.githubusercontent.com/10780778/35972123-37bb9d9a-0cd9-11e8-94cd-9fdf324a5411.png)


are given by 

![begineqnarray-k_1-ft_n-y_n-k_2-ft_n-fracdelta-35d808c6ef](https://user-images.githubusercontent.com/10780778/35971655-8a34be0a-0cd7-11e8-925d-6cb110703901.png)

For a similar adaptive method using the fifth order Cash-Karp Runge-Kutta method with fourth order embedded error estimator
