<a id="readme-top"></a>

<br />
<div align="center">

  <h3 align="center">2D Physics Engine</h3>

  <p align="center">
    A two dimensional physics engine featuring accurate collisions between rigid bodies 
    <br />
    <br />
  </p>
</div>

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

Hello, This is a simple 2D physics engine using No-Velocity Verlet Integration
I created for fun one summer with an external library called SFML in order
to easily handle visuals as well as input from the keyboard during the
programme's runtime. The code allows the user to simulate rigid shapes
colliding with each other as well as pairs of circles connected by springs.

It was inspired by the work done by [johnBuffer](https://github.com/johnBuffer/VerletSFML) here on GitHub.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Built With

This project was built using SFML.

In order to run this code SFML must be installed first.

I would recommend the following quick tutorial by [EOD - Ethan](https://www.youtube.com/watch?v=lFzpkvrscs4) on YouTube. 

From here it should be straightforward. Just running the code in Visual Studio should bring up the following screen:

<img src="/README%20images/emptyScene.png" alt="Image of an empty scene." width="750"/>

* [![SFML][SFML-logo]][SFML-url]

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- USAGE EXAMPLES -->
## Usage

This 2D physics engine is pretty straightforward. The primary ways of interacting with it is by adding more shapes to the scene.

Shapes can be added using the corresponding keys that can be found towards the top of the display.

Once a shape is added, it will appear somewhere along the top of the display, with a randomly selected colour, and will then fall with the constant gravity present in the scene.

<img src="/README%20images/sceneWithShapes.png" alt="Image of a scene with a few shapes in it." width="750"/>

Additionally, there are two more structures that are present in the physics engine: springs and sponges with sponges just being a circles connected by a network of springs. They can be added using "F5" and "F6" respectively.

The exact implementation of these is currently a "work in progress" as such, the keys to add them are not displayed but they can still be added nonetheless.

<img src="/README%20images/spongeEmptyScene.png" alt="Image of a scene with a single sponge in it." width="750"/>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- CONTACT -->
## Contact

Project Link: [https://github.com/Joseph-RF/2DPhysicsEngine](https://github.com/Joseph-RF/2DPhysics)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

Find below a list of resources I found helpful during the development of this project.

* [SFML Documentation](https://www.sfml-dev.org/documentation/2.6.2/)
* [Pikuma's SAT Youtube Tutorial](https://www.youtube.com/watch?v=-EsWKT7Doww)
* [Two-Bit Coding's "Let's Make a Physics Engine" series](https://www.youtube.com/watch?v=lzI7QUyl66g&list=PLSlpr6o9vURwq3oxVZSimY8iC-cdd3kIs)
* [Pezzza's Work "collision detection optimisation Youtube video](https://www.youtube.com/watch?v=lzI7QUyl66g&list=PLSlpr6o9vURwq3oxVZSimY8iC-cdd3kIs)
* [Badges for README](https://github.com/alexandresanlim/Badges4-README.md-Profile)
* [README Template](https://github.com/othneildrew/Best-README-Template)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->

[SFML-url]: https://www.sfml-dev.org/
[SFML-logo]: https://img.shields.io/badge/SFML-8CC445?style=for-the-badge&logo=sfml&logoColor=white