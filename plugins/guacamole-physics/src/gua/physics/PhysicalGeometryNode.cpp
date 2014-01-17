// class header
#include <gua/physics/PhysicalGeometryNode.hpp>


namespace gua{


PhysicalGeometryNode::PhysicalGeometryNode(/*std::string const& name,
				 physics::Physics* physics,
                 GeometryNode::Configuration const& configuration,
                 math::mat4 const& transform,*/
                 std::shared_ptr<GeometryNode> const& geom,
                 physics::Physics* physics,
				 std::shared_ptr<physics::CollisionShapeNode> const& cs,
                 float mass,
                 float friction,
                 float restitution)
		: 	//GeometryNode(name,configuration,transform),
			TransformNode(geom->get_name()+"_physical"),
			rigid_body_(nullptr),
			mass_(mass),
			friction_(friction),
			restitution_(restitution),
			collision_shape_(cs),
			physics_(physics),
			geometry_(geom)
		{
			std::cout<<get_name()<<std::endl;
			add_child(geometry_);
		}



bool
PhysicalGeometryNode::make_collidable(bool b_make_collidable,bool warn_parent){

	if(b_make_collidable){

		if(rigid_body_==nullptr){

			if (collision_shape_ == nullptr){
				//TODO compute collision shape automatically here!
				std::cout<<"Missing CollisionShape in make_collidable()!!!"<<std::endl;
				return false;
			}

			rigid_body_= std::shared_ptr<physics::RigidBodyNode>(new physics::RigidBodyNode(get_name()+"_rb_",mass_,friction_,restitution_,get_world_transform()));

			//rigid_body_->add_child(collision_shape_);

			auto collision_shapes = std::list<std::pair<std::shared_ptr<physics::CollisionShapeNode>,math::mat4>>();

			collect_collision_shapes(this,collision_shapes);

			std::cout<<get_name()<<std::endl;
			//collision_shapes.push_back(std::make_pair<std::shared_ptr<physics::CollisionShapeNode>,math::mat4>(get_collision_shape(),get_world_transform()));

			std::cout<<"make_collidable"<<std::endl;
			std::cout<<"collision_shapes lenght "<<collision_shapes.size()<<std::endl;

			for(auto cs : collision_shapes){
				std::cout<<"add collision shape here!!!!!!"<<std::endl;
				cs.first->set_transform(scm::math::inverse(get_world_transform()) * cs.second);
				rigid_body_->add_child(cs.first);
			}

			physics_->add_rigid_body(rigid_body_);
			if(warn_parent)warn_parent_physics(get_parent_shared());
		}
		else{
			//make_collidable(false,false);
			//return make_collidable(true,false);
			return update_physics_structure();
		}
	}
	else if(rigid_body_){
		physics_->remove_rigid_body(rigid_body_);
		rigid_body_ = nullptr;
		if(warn_parent)warn_parent_physics(get_parent_shared());
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


void
PhysicalGeometryNode::set_world_transform(math::mat4 const& transform){
	auto parent = get_parent();
    set_transform(scm::math::inverse(parent->get_world_transform())* transform);
}



std::shared_ptr<physics::CollisionShapeNode>
PhysicalGeometryNode::get_collision_shape() const{
	if (collision_shape_){return collision_shape_;}
	else {return nullptr;}
}


void
PhysicalGeometryNode::collect_collision_shapes(Node* node,std::list<std::pair<std::shared_ptr<physics::CollisionShapeNode>,math::mat4>>& collision_shapes)const{
	if(dynamic_cast<PhysicalGeometryNode*>(node)){
		auto phys_node = dynamic_cast<PhysicalGeometryNode*>(node);
		if(!phys_node -> is_collidable()){
			for(auto const& child : phys_node->get_children()){
				collect_collision_shapes(&*child,collision_shapes);
			}
			if(phys_node->get_collision_shape()){
				collision_shapes.push_back(std::make_pair(phys_node->get_collision_shape(),phys_node->get_world_transform()));
			}
		}
	}
	else{
		for(auto child : node->get_children()){
			collect_collision_shapes(&*child,collision_shapes);
		}
	}
}

void
PhysicalGeometryNode::warn_parent_physics(std::shared_ptr<Node> const& parent)const{
	if(std::dynamic_pointer_cast<PhysicalGeometryNode>(parent)){
		std::shared_ptr<PhysicalGeometryNode> phys_node = std::dynamic_pointer_cast<gua::PhysicalGeometryNode>(parent);
		if(phys_node->is_collidable()){
			if(!phys_node->update_physics_structure()){
				std::cout<<"Error in Update Phyics Structure of Parent!"<<std::endl;
			}
		}
		else if(parent->get_parent_shared()){
			warn_parent_physics(parent->get_parent_shared());
		}
	}
	else if(parent->get_parent_shared()){
		warn_parent_physics(parent->get_parent_shared());
	}

}

bool
PhysicalGeometryNode::update_physics_structure(){
	make_collidable(false,false);
	return make_collidable(true,false);
}




/* virtual */ /*void PhysicalGeometryNode::accept(NodeVisitor& visitor) {

  visitor.visit(this);
}//=???????????????????????????


std::shared_ptr<Node> PhysicalGeometryNode::copy() const {
  return std::make_shared<PhysicalGeometryNode>(geometry_,physics_,collision_shape_);
}//=??????????????????????????

*/


}