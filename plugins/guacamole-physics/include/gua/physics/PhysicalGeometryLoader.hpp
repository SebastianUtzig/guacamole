#ifndef GUA_PHYSICAL_GEOMETRY_LOADER_HPP
#define GUA_PHYSICAL_GEOMETRY_LOADER_HPP

// guacamole headers
#include <gua/guacamole.hpp>
#include <gua/physics.hpp>

namespace gua{

class PhysicalGeometryLoader : public NodeVisitor{

  public:

  	PhysicalGeometryLoader(physics::Physics*);

  	virtual ~PhysicalGeometryLoader();


    void                    visit(GeometryNode*);

    void                    visit(Node*);

    std::shared_ptr<Node>   create_physical_objects_from_file
                            (std::string const& node_name,
                            std::string const& file_name,
                            std::string const& fallback_material,
                            float mass = 1.0
                            );



  private:

    physics::Physics* physics_;
    float             mass_;

    GeometryLoader  loader_;






};

}

#endif