# Pathfinding, Physics & 2D Gameplay - C++

A study project built over 8 weeks where I implemented A* pathfinding, a triangulated navigation mesh, and a 2D physics engine from scratch in C++, then combined them into a playable game.

The base engine was provided by Breda University of Applied Sciences and is not included in this repository. It provided a basic PBR renderer capable of loading and rendering GLTF models, an inspector, and an Entity-Component-System framework.

The files in this repository are the systems and gameplay code I wrote on top of it: pathfinding, physics, collision, AI, and gameplay logic, all built within the ECS framework.

---

## What's in this repo

### AI and pathfinding (`engine-extension/ai/`)
The systems I built for pathfinding and navigation:

- `graph` - graph data structure and A* search implementation with a custom min-heap priority queue
- `map` - loads map data (walkable area, obstacles, agent/player positions) from a text file
- `NavigationMesh` - builds a walkable navmesh using Clipper2 for polygon boolean subtraction and CDT for constrained Delaunay triangulation, then converts the result into a graph A* can navigate
- `navigation_system` - ECS system running on a fixed timestep (10 fps) that moves agents along computed paths using look-ahead steering
- `agent_components` - `NavigationComponent` data component for agents

### Physics (`engine-extension/physics/`)
A custom 2D physics engine:

- `physics_system` - fixed-timestep physics loop with gravity, circle-circle and circle-polygon collision detection, and impulse-based resolution with coefficient of restitution. Supports static, dynamic, and kinematic body types
- `physics_components` - `BodyComponent` and `DiskColliderComponent` data components, with `CollisionData` interface for reading collision results in game code

### Game (`game/`)
Gameplay layer built on top of the engine systems:

- `hello` - entry point that initializes systems, creates the player entity, camera, lighting, and spawns agents from the map file
- `player_input_system` - WASD movement, mouse-to-world unprojection for aiming, left click to fire, space to send agents toward the player
- `game_objects_system` - bullet spawning and bounce logic; bullets bounce off walls and destroy agents on hit
- `game_components` - `PlayerComponent` and `BulletComponent` data components

### Map files (`game/maps/`)
Text-based map format defining bounding boxes, obstacle polygons, player start position, and agent spawn positions. The navmesh is generated from this data at runtime.

---

## How the systems connect

1. `map` loads the level from a text file
2. `NavigationMesh` takes the map data, subtracts obstacles from the walkable area, triangulates the result, and builds a graph
3. `navigation_system` runs A* on that graph each time a new path is requested, then steers agents along the result each tick
4. `physics_system` handles movement integration, collision detection, and resolution - keeping physics decoupled from both the navmesh and the game layer
5. Game code reads collision data from `DiskColliderComponent` to implement bullet logic.

---

## Dependencies (not included)

The project uses the following third-party libraries which were part of the engine setup:

- [GLM](https://github.com/g-truc/glm) - math
- [EnTT](https://github.com/skypjack/entt) - ECS
- [Clipper2](https://github.com/AngusJohnson/Clipper2) - polygon boolean operations
- [CDT](https://github.com/artem-ogre/CDT) - constrained Delaunay triangulation

---

## Portfolio

More details about this project, including gifs and explanation of the design decisions, on my portfolio:  
https://danielkocan.github.io/projects/2-pathfinding-and-collisions

---