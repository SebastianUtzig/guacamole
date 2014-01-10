// class header
#include <PhysicalGeometryNode.hpp>


namespace gua{


PhysicalGeometryNode::PhysicalGeometryNode(std::string const& name,
                 GeometryNode::Configuration const& configuration,
                 math::mat4 const& transform,
                 float mass,
                 float friction,
                 float restitution,
				 std::shared_ptr<physics::CollisionShapeNode> cs)
		: 	GeometryNode(name,configuration,transform),
			rigid_body_(nullptr),
			mass_(mass),
			friction_(friction),
			restitution_(restitution),
			collision_shape_(cs)
		{}



bool
PhysicalGeometryNode::make_collidable(bool make_collidable,physics::Physics* physics){

	if(make_collidable and rigid_body_==nullptr){

		rigid_body_= std::shared_ptr<physics::RigidBodyNode>(new physics::RigidBodyNode(Node::get_name()+"_rb_",mass_,friction_,restitution_,Node::get_world_transform()));

		if (collision_shape_){
			rigid_body_->add_child(collision_shape_);
		}
		else{
			//TODO compute collision shape automatically here!
			std::cout<<"Missing CollisionShape in make_collidable()!!!"<<std::endl;
			return false;
		}

		physics->add_rigid_body(rigid_body_);
	}
	else{
		physics->remove_rigid_body(rigid_body_);
		rigid_body_ = nullptr;
	}

	return true;

}


bool
PhysicalGeometryNode::is_collidable()const{
	if (rigid_body_){
		return true;
	}
	return false;
}



}