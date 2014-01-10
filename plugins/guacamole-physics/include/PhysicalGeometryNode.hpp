#ifndef GUA_PHYSICAL_GEOMETRY_NODE_HPP
#define GUA_PHYSICAL_GEOMETRY_NODE_HPP


// guacamole headers
//#include <gua/scenegraph/GeometryNode.hpp>
#include <gua/guacamole.hpp>
#include <gua/physics.hpp>
//#include <gua/physics/CollisionShapeNode.hpp>
//#include <gua/physics/RigidBodyNode.hpp>

// external headers
#include <string>

namespace gua{
	

class GUA_DLL PhysicalGeometryNode: public GeometryNode{

  public:

  	struct Configuration {
      GUA_ADD_PROPERTY(std::string,     geometry,   "gua_default_geometry");
      GUA_ADD_PROPERTY(std::string,     material,   "gua_default_material");
    };

    Configuration data;

    PhysicalGeometryNode() {};

  	PhysicalGeometryNode(std::string const& name,
                 GeometryNode::Configuration const& configuration = GeometryNode::Configuration(),
                 math::mat4 const& transform = math::mat4::identity(),
                 float mass = 0,
                 float _friction = 0,
                 float _restitution = 0,
                 std::shared_ptr<physics::CollisionShapeNode> cs = nullptr);

  	bool 	make_collidable(bool,physics::Physics*);
  	bool	is_collidable() const;



  private:

  	std::shared_ptr<physics::RigidBodyNode>      rigid_body_;
  	std::shared_ptr<physics::CollisionShapeNode> collision_shape_;
  	float				                                 mass_;
  	float				                                 friction_;
  	float				                                 restitution_;






};

}

#endif  // GUA_GEOMETRY_NODE_HPP