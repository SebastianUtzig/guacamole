#ifndef GUA_PHYSICAL_NODE_HPP
#define GUA_PHYSICAL_NODE_HPP


// guacamole headers
#include <gua/guacamole.hpp>
#include <gua/physics.hpp>
#include <gua/physics/CollisionShapeCollector.hpp>

// external headers
#include <string>
#include <utility>

namespace gua{
	

class GUA_DLL PhysicalNode: public TransformNode{

  public:

    PhysicalNode() {};

  	PhysicalNode(std::shared_ptr<GeometryNode> const&,
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


    /* virtual */ void accept(NodeVisitor&);

    /* virtual */ void scale(float x, float y, float z);

    /* virtual */ void translate(float x, float y, float z);



  private:

    void collect_collision_shapes(Node*,std::list<std::tuple<std::shared_ptr<physics::CollisionShapeNode>,math::mat4,float>>&)const;
    void warn_parent_physics(Node*)const;
    bool update_physics_structure();
    void calculate_collision_shape();

  	
    physics::Physics*                            physics_;
    std::shared_ptr<physics::RigidBodyNode>      rigid_body_;
  	std::shared_ptr<physics::CollisionShapeNode> collision_shape_;
    std::shared_ptr<GeometryNode>                geometry_;

    CollisionShapeCollector*                     cs_collector_;

  	float				                                 mass_;
  	float				                                 friction_;
  	float				                                 restitution_;

    math::vec3                                   scale_;
    bool                                         set_scale_;


};

}

#endif