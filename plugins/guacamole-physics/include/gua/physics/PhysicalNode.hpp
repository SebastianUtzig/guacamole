#ifndef GUA_PHYSICAL_NODE_HPP
#define GUA_PHYSICAL_NODE_HPP


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
	

class GUA_DLL PhysicalNode: public TransformNode{

  public:

  	/*struct Configuration {
      GUA_ADD_PROPERTY(std::string,     geometry,   "gua_default_geometry");
      GUA_ADD_PROPERTY(std::string,     material,   "gua_default_material");
    };*/

    //Configuration data;

    PhysicalNode() {};

  	PhysicalNode(/*std::string const& name,
                 physics::Physics* physics,
                 GeometryNode::Configuration const& configuration = GeometryNode::Configuration(),
                 math::mat4 const& transform = math::mat4::identity(),*/
                 std::shared_ptr<GeometryNode> const&,
                 physics::Physics* physics,
                 std::shared_ptr<physics::CollisionShapeNode> const& cs = nullptr,
                 float mass = 0.5f,
                 float friction = 2.9f,
                 float restitution = 0.3f);

  	bool 	make_collidable(bool,bool warn_parent = true);
  	bool	is_collidable() const;

    void  set_world_transform(math::mat4 const&);
    std::shared_ptr<physics::CollisionShapeNode>  get_collision_shape();
    std::shared_ptr<GeometryNode>  get_geometry()const;

    float get_mass()const;

    /*virtual*/ math::mat4 get_world_transform() const;

    /*virtual*/ void update_cache();


    ///* virtual */ void accept(NodeVisitor&);



  private:

    //std::shared_ptr<Node> copy() const;

    void collect_collision_shapes(Node*,std::list<std::tuple<std::shared_ptr<physics::CollisionShapeNode>,math::mat4,float>>&)const;
    void warn_parent_physics(Node*)const;
    bool update_physics_structure();
    void calculate_collision_shape();

  	
    physics::Physics*                            physics_;
    std::shared_ptr<physics::RigidBodyNode>      rigid_body_;
  	std::shared_ptr<physics::CollisionShapeNode> collision_shape_;
    std::shared_ptr<GeometryNode>                geometry_;
  	float				                                 mass_;
  	float				                                 friction_;
  	float				                                 restitution_;

    math::vec3                                   scale_;
    bool                                         set_scale_;
    math::mat4                                   geom_world_;
    //math::mat4                                   parent_world_;

    //math::mat4                                   last_simulation_step_;






};

}

#endif