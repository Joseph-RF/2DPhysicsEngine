# 2DPhysicsEngine

## Description
Hello! This is a simple 2D physics engine using No-Velocity Verlet Integration I created for fun one summer with an external library called SFML in order to easily handle visuals as well as input from the keyboard during the programme's runtime. The code allows you to simulate rigid spheres colliding with each other as well as pairs of these same spheres connected by springs.

It is inspired by the work done by [johnBuffer](https://github.com/johnBuffer/VerletSFML) here on GitHub.

P.S. This is my first attempt at using Git with a project and pushing it to GitHub. Excuse the ugly README.

## Instructions
*Requirements*

In order to run this code you will first need to install SFML.

I would recommend the following quick tutorial by [EOD - Ethan](https://www.youtube.com/watch?v=lFzpkvrscs4) on YouTube. 

From here it should be straightforward. Just run the code in Visual Studio and the following empty scene should appear:

![Image of an empty scene](/README%20images/EmptyScene.png)

## Features

For now, this project features two main entities: Simple rigid spheres and pairs of the same spheres attached by a spring.

They can be added to the scene by simply pressing "s" and "r" on the keyboard respectively.

![Image of a scene with a sphere and a spring in it](/README%20images/sphereWithSpringScene.png)

The main interaction in this engine is the collision between entities which is governed by standard physics. The mechanics can be adjusted by changing the value for the coefficient of resitution.

## Updates

Added an object called "Sponge" composed of several entities interconnected by springs resulting in a soft-body-like object. For now it is just 5 entities and 8 springs. Below, an image of a "sponge"

![Image of a sponge](/README%20images/spongeEmptyScene.png)

Massive changes done to the way the physics is computed to ensure a no-velocity verlet integration as this would provide stability to the simulation.

Improved the simulation efficiency by subdividing the "box" increasing the number of entities that can be simulated by around 6x. 