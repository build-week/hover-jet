Viewer -- CPlotPlot
===================

Visualization in C++ is very frustrating. There are few idiomatic C++11 libraries that just let you throw down objects into a gl window without much thought.

Viewer (CPlotPlot) aims to achieve this. It's a super quick way to throw arbitrary `primitives` into a view.

TODO:
* Add an overload for add_primitive:
    `void add_primitive(std::shared_ptr<Primitive>, SE3 primitive_from_world);`
    Which generates a frame

* Add an overload for add_primitive in Window2d:
    `void add_primitive(std::shared_ptr<Primitive>, SE2 primitive_from_world);`
    Which generates a frame, by se3ing our se2

* Add primitives (like images) that exist and work by default in 2d
* Add primitives (like 2d plots) that exist and work by default in 2d
* Add some mechanism for drawing these things in 3d
* Reduce the overhead for adding primitives -- right now it's a lot of mungling
* Add basic interaction and interaction widgets
* Add basic scene graph features
