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
	
/**
 * This class contains essential informations enabling
 *  physical simulations in a subgraph.
 *
 */
class GUA_DLL PhysicalNode: public TransformNode{

  public:

  /**
   * Constructor.
   *
   * This constructs a PhycialNode.
   *
   * \param child               PhysicalNode will be parent of this node.
   * \param physics             Instance of physics world is needed for simulations.
   * \param collision_shape     Certain CollisionShapeNode can be given as representation for GeometryNode childs
   *                            - otherwise the shapes will be computed automatically.
   * \param mass                Mass of simulated GeometryNode childs.
   * \param friction            Friction of simulated GeometryNode childs.
   * \param restitution         Restitution of simulated GeometryNode childs.
   */
    PhysicalNode(std::shared_ptr<Node> const& child,
                 physics::Physics* physics,
                 std::shared_ptr<physics::CollisionShapeNode> const& collision_shape = nullptr,
                 float mass = 0.5f,
                 float friction = 2.9f,
                 float restitution = 0.3f);

  	
    /**
     * Starts/stops physical simulation of this subgraph
     * 
     * \param simulate          Start/stop.
     * \param warn_parent       Just for internal usage.
     */   
    bool 	simulate(bool simulate,bool warn_parent = true);

  	bool	is_simulating() const;

    /**
     * Will be used by gua::Physics instance while simulation
     *  to transform this subgraph in world coordinate system.
     *
     * \param world_transform   Matrix that will be used as local and world transform at the same time.
     */  
    void  set_world_transform(math::mat4 const& world_transform);

    /**
     * Returns the CollisionShapeNode given by user.
     */  
    std::shared_ptr<physics::CollisionShapeNode>  get_collision_shape();
    
    /**
     * Returns the RigidBodyNode (physical representation of this node) when simulation is running.
     * You can specify advanced physical properties on this object! 
     */ 
    std::shared_ptr<physics::RigidBodyNode>  get_rigid_body()const;


    /**
     * Returns the mass each GeometryNode child has.
     */  
    float get_mass()const;

    /**
     * Changes mass of physical representation (RigidBodyNode).
     *
     * \param mass  New mass to be set. 
     *
     * If mass is 0 -> Simulated object will be static.
     * If mass is > 0 -> Simulated object will be dynamic.
     */ 
    void set_mass(float mass);

    /**
     * Will ignore parent transformations if simulation is running!!!
     * (This is because simulated objects must be in the same coordinate system.)
     */  
    /*virtual*/ math::mat4 get_world_transform() const;

    /**
     * Updates world transform differently if simulation is running (ignore parent transformations)!!!
     * (This is because simulated objects must be in the same coordinate system.)
     * Called every frame.
     */  
    /*virtual*/ void update_cache();


    /**
     * Accepts a visitor and calls concrete visit method
     *
     * This method implements the visitor pattern for Nodes
     *
     */
    /* virtual */ void accept(NodeVisitor&);

    /**
     * You can transform this node also while simulation.
     * Scale will not change collision shape given by user.
     */  
    /* virtual */ void scale(float x, float y, float z);

    /* virtual */ void rotate(float angle, float x, float y, float z);

    /* virtual */ void translate(float x, float y, float z);




  private:

    void warn_parent_physics(Node*)const;
    bool update_physics_structure();

  	
    physics::Physics*                            physics_;
    std::shared_ptr<physics::RigidBodyNode>      rigid_body_;
  	std::shared_ptr<physics::CollisionShapeNode> collision_shape_;
    std::shared_ptr<Node>                        child_;

    CollisionShapeCollector*                     cs_collector_;

    math::vec3                                   center_of_mass_;

  	float				                                 mass_;
  	float				                                 friction_;
  	float				                                 restitution_;

    math::vec3                                   scale_;
    bool                                         set_scale_;

};

}

#endif