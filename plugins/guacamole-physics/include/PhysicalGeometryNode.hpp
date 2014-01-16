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
#include <utility>

namespace gua{
	

class GUA_DLL PhysicalGeometryNode: public GeometryNode{

  public:

  	/*struct Configuration {
      GUA_ADD_PROPERTY(std::string,     geometry,   "gua_default_geometry");
      GUA_ADD_PROPERTY(std::string,     material,   "gua_default_material");
    };*/

    //Configuration data;

    PhysicalGeometryNode() {};

  	PhysicalGeometryNode(std::string const& name,
                 physics::Physics* physics,
                 GeometryNode::Configuration const& configuration = GeometryNode::Configuration(),
                 math::mat4 const& transform = math::mat4::identity(),
                 float mass = 0.5f,
                 float friction = 2.9f,
                 float restitution = 0.3f,
                 std::shared_ptr<physics::CollisionShapeNode> cs = nullptr);

  	bool 	make_collidable(bool,bool warn_parent = true);
  	bool	is_collidable() const;

    void  set_world_transform(math::mat4 const&);
    std::shared_ptr<physics::CollisionShapeNode>  get_collision_shape() const;



  private:

    void collect_collision_shapes(std::shared_ptr<Node>,std::list<std::pair<std::shared_ptr<Node>,math::mat4>>&)const;
    void warn_parent_physics(std::shared_ptr<Node>)const;
    bool update_physics_structure();

  	
    physics::Physics*                            physics_;
    std::shared_ptr<physics::RigidBodyNode>      rigid_body_;
  	std::shared_ptr<physics::CollisionShapeNode> collision_shape_;
  	float				                                 mass_;
  	float				                                 friction_;
  	float				                                 restitution_;






};

}

#endif  // GUA_GEOMETRY_NODE_HPP