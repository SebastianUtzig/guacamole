// class header
#include <gua/physics/PhysicalGeometryLoader.hpp>


namespace gua{


PhysicalGeometryLoader::PhysicalGeometryLoader(physics::Physics * ph)
		:physics_(ph),
		 make_all_collidable_(false),
		 loaded_nodes_(){}

PhysicalGeometryLoader::~PhysicalGeometryLoader(){
	physics_ = nullptr;
	while(!loaded_nodes_.empty()){
		loaded_nodes_.pop_back();
	}
}


void
PhysicalGeometryLoader::visit(GeometryNode* geom){
	
	std::cout<<"visit geometry node!: "<<std::endl;


	//put physical node in between geom and parent
	auto parent = geom->get_parent();

	std::cout<<"1: "<<std::endl;
	parent->clear_children();
	std::cout<<"2: "<<std::endl;

	std::shared_ptr<GeometryNode>shared_geom(geom);
	std::cout<<"3: "<<std::endl;

	auto physical_node = new gua::PhysicalNode(shared_geom,physics_,nullptr,1.0);
	std::cout<<"4: "<<std::endl;

	if(make_all_collidable_){
		physical_node->make_collidable(true);
	}
	std::cout<<"5: "<<std::endl;

	std::shared_ptr<Node>shared_node(physical_node);
	std::cout<<"6: "<<std::endl;
				
	loaded_nodes_.push_back(shared_node);
	std::cout<<"7: "<<std::endl;

	parent->add_child(shared_node);
	std::cout<<"8: "<<std::endl;

	for(auto child : geom->get_children()){
		child->accept(*this);
	}
	std::cout<<"9: "<<std::endl;


}

void
PhysicalGeometryLoader::visit(Node* node){
	std::cout<<"visit node!: "<<std::endl;
	auto geom = dynamic_cast<GeometryNode*>(node);
	if(geom){
		visit(geom);
	}
	else{	
		for(auto child : node->get_children()){
			child->accept(*this);
		}
	}
}

std::shared_ptr<Node>
PhysicalGeometryLoader::create_physical_objects_from_file(
						  std::string const& node_name,
                          std::string const& file_name,
                          std::string const& fallback_material,
                          bool make_collidable
                          )
{
	auto node = loader_.create_geometry_from_file(node_name, file_name, fallback_material);


	if(node){
		if(node->get_children().size() == 0){

			std::shared_ptr<gua::GeometryNode> geometry = std::dynamic_pointer_cast<gua::GeometryNode>(node);
			if(geometry){
				//geometry->data.set_geometry("");
				//auto phys_node2 = new gua::PhysicalNode(geometry,&physics,csn2);
				auto physical_node = new gua::PhysicalNode(geometry,physics_,nullptr,1.0);

				if(make_collidable){
					physical_node->make_collidable(true);
				}

				std::shared_ptr<Node>return_node(physical_node);
				
				loaded_nodes_.push_back(return_node);
				
				return return_node;
			}
			else{
				std::cout<<"Error in PhysicalGeometryLoader: Single loaded node isnt a GeometryNode!"<<std::endl;
			}

		}
		else{
			//geometry group
			make_all_collidable_ = make_collidable;

			check(&*node);

			std::cout<<"10: "<<std::endl;

			return node;
		}

	}

	
}

void
PhysicalGeometryLoader::check(Node* node){

	//node always group node ???
	std::cout<<"check group node: "<<node->get_name()<<std::endl;
	std::cout<<"group node children size: "<<node->get_children().size()<<std::endl;

	for(auto child : node->get_children()){
		std::cout<<"child path: "<<child->get_path()<<std::endl;
		std::cout<<"child name: "<<child->get_name()<<std::endl;
		if(child->get_path() != "/"){
			child->accept(*this);
		}
		/*auto geom = std::dynamic_pointer_cast<GeometryNode>(child);
		if(geom){

			visit(dynamic_cast<GeometryNode*>(&*geom));
		}*/
	}

}




}
