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

  	PhysicalNode(//std::shared_ptr<GeometryNode> const&,
                 std::shared_ptr<Node> const&,
                 physics::Physics* physics,
                 std::shared_ptr<physics::CollisionShapeNode> const& cs = nullptr,
                 float mass = 0.5f,
                 float friction = 2.9f,
                 float restitution = 0.3f);

  	bool 	simulate(bool,bool warn_parent = true);
  	bool	is_simulating() const;

    void  set_world_transform(math::mat4 const&);
    
    //void cs_already_simulated_in(PhysicalNode*);

//    std::shared_ptr<physics::CollisionShapeNode>  get_collision_shape();
//    std::shared_ptr<GeometryNode>  get_geometry()const;
    std::shared_ptr<physics::RigidBodyNode>  get_rigid_body()const;


    float get_mass()const;

    void set_mass(float mass);

    /*virtual*/ math::mat4 get_world_transform() const;

    /*virtual*/ void update_cache();


    /* virtual */ void accept(NodeVisitor&);

    /* virtual */ void scale(float x, float y, float z);

    /* virtual */ void rotate(float angle, float x, float y, float z);

    /* virtual */ void translate(float x, float y, float z);

    // appending childs just under geometry (phys node and geometry must be together)
    /*template<typename T>
    std::shared_ptr<T> add_child(std::shared_ptr<T> const& new_node) {
      return geometry_->add_child(new_node);
    }*///cant be overwritten!!!!



  private:

    void collect_collision_shapes(Node*,std::list<std::tuple<std::shared_ptr<physics::CollisionShapeNode>,math::mat4,float>>&)const;
    void warn_parent_physics(Node*)const;
    bool update_physics_structure();
//    void calculate_collision_shape();

  	
    physics::Physics*                            physics_;
    std::shared_ptr<physics::RigidBodyNode>      rigid_body_;
  	std::shared_ptr<physics::CollisionShapeNode> collision_shape_;
    //std::shared_ptr<GeometryNode>                geometry_;
    std::shared_ptr<Node>                        child_;

    CollisionShapeCollector*                     cs_collector_;

    //PhysicalNode*                                cs_already_simulated_in_;

  	float				                                 mass_;
  	float				                                 friction_;
  	float				                                 restitution_;

    math::vec3                                   scale_;
    bool                                         set_scale_;


  public:

    //RigidBody Interface (only if this node is collidable):
    ////////////////////////////////////////////

    /**
     * Makes the rigid body kinematic.
     *
     * Movements of kinematic rigid bodies are controlled by the user,
     * physical simulation will not move such objects.
     * Kinematic bodies can influence other dynamic rigid bodies, but there is
     * no collision detection with static bodies.
     *
     * When the body becomes kinematic it can be moved by the user using
     * set_transform(), rotate(), and translate() methods.
     *
     * \param enabled If true, the body is kinematic; otherwise either static
     *                or dynamic.
     */
//    void set_kinematic(bool kinematic);

    /**
     * Checks if the rigid body is kinematic.
     *
     * \return True if the rigid body is kinematic.
     */
//    bool is_kinematic() const { return rigid_body_->is_kinematic(); }

    /**
     * Sets the new mass of the body.
     *
     * Setting the mass to zero makes the rigid body static.
     *
     * \param mass New mass.
     */
//    void set_mass(float mass);

    /**
     * Gets current mass of the rigid body.
     *
     * The mass of zero denotes non-dynamic rigid bodies.
     *
     * \return Mass value.
     */
//    float mass() const { return mass_; }

    /**
     * Sets the friction of the rigid body.
     *
     * Friction applies only when rigid bodies are in contact.
     *
     * A value of zero means that the body has no friction at all.
     *
     * \param frict New friction.
     * \sa    set_rolling_friction()
     */
//    void set_friction(float frict);

    /**
     * Gets current friction of the rigid body.
     *
     * \return Friction factor.
     */
//    float friction() const { return rigid_body_->friction(); }

    /**
     * Sets the rolling friction of the rigid body.
     *
     * Rolling friction hampers in rolling of curved bodies (even on sloped
     * surfaces). Please note that the rolling friction should be set on all
     * interacting bodies to come into effect.
     *
     * \param frict New rolling friction.
     * \sa    set_friction()
     */
//    void set_rolling_friction(float frict);

    /**
     * Gets current rolling friction of the rigid body.
     *
     * \return Rolling friction factor.
     */
//    float rolling_friction() const { return rigid_body_->rolling_friction(); }

    /**
     * Sets the restitution of the rigid body.
     *
     * Restitution indicates bounciness of the body. It shows how much
     * velocity is lost after collision. With a value of zero all velocity is
     * lost preventing any bouncing.
     *
     * \param rest New restitution of the body.
     */
//    void set_restitution(float rest);

    /**
     * Gets current restitution of the rigid body.
     *
     * \return Restitution factor.
     */
//    float restitution() const { return rigid_body_->restitution(); }

    /**
     * Sets the linear and angular damping coefficients.
     *
     * Damping expresses velocity attenuation. It indicates how velocity
     * (linear and angular) will be decreasing in time. In contrast to
     * friction, damping does not require bodies being in contact.
     *
     * \param lin_damping Linear damping of the body.
     * \param ang_damping Angular damping of the body.
     * \sa    set_friction()
     */
//    void set_damping(float lin_damping, float ang_damping);

    /**
     * Gets current linear velocity damping.
     *
     * \return Damping factor.
     */
//    float linear_damping() const { return rigid_body_->linear_damping(); }

    /**
     * Gets current angular velocity damping.
     *
     * \return Damping factor.
     */
//    float angular_damping() const { return rigid_body_->angular_damping(); }

    /**
     * Applies the force to the rigid body at a given position.
     *
     * If the position is not at the center of the body, torque is
     * also applied.
     *
     * \param torque  The vector containing the force values.
     * \param rel_pos The vector containing the relative point of
     *                force application.
     * \sa    apply_central_force(), apply_torque()
     */
//    void apply_force(const math::vec3& force, const math::vec3& rel_pos);

    /**
     * Applies the force at the rigid body center.
     *
     * \param force The vector containing the force valus.
     * \sa    apply_force()
     */
//    void apply_central_force(const math::vec3& force);

    /**
     * Applies the torque to the rigid body.
     *
     * \param torque The vector containing the torque values.
     */
//    void apply_torque(const math::vec3& torque);

    /**
     * Applies the torque impulse to the rigid body.
     *
     * \param torque The vector containing the torque impulse values.
     * \sa    apply_impulse()
     */
//    void apply_torque_impulse(const math::vec3& torque);

    /**
     * Applies an impulse to the rigid body at a given position.
     *
     * If the position is not at the center of the body, torque impulse is
     * also applied.
     *
     * \param impulse  The vector containing the impulse values.
     * \param rel_pos  The vector containing the relative point of
     *                 impulse application.
     * \sa    apply_central_impulse(), apply_torque_impulse()
     */
//    void apply_impulse(const math::vec3& impulse, const math::vec3& rel_pos);

    /**
     * Applies an impulse at the rigid body center.
     *
     * \param torque The vector containing the impulse values.
     * \sa    apply_impulse()
     */
//    void apply_central_impulse(const math::vec3& impulse);

    /**
     * Clears all forces and torques.
     */
//    void clear_forces();

    /**
     * Sets angular velocity of the rigid body.
     *
     * \param vel The vector containing new velocity values.
     * \sa    set_linear_velocity()
     */
//    void set_angular_velocity(const math::vec3& vel = math::vec3());

    /**
     * Gets current angular velocity of the rigid body.
     *
     * \return Current velocity.
     * \sa     linear_velocity()
     */
//    math::vec3 angular_velocity() const;

    /**
     * Sets linear velocity of the rigid body.
     *
     * \param vel The vector containing new velocity values.
     * \sa    set_angular_velocity()
     */
//    void set_linear_velocity(const math::vec3& vel = math::vec3());

    /**
     * Gets current linear velocity of the rigid body.
     *
     * \return Current velocity.
     * \sa     angular_velocity()
     */
//    math::vec3 linear_velocity() const;


};

}

#endif