// class header
#include <gua/physics/PhysicalGeometryLoader.hpp>


namespace gua{


PhysicalGeometryLoader::PhysicalGeometryLoader(physics::Physics * ph)
		:physics_(ph),
		 make_all_collidable_(false),
		 loaded_nodes_(){}

PhysicalGeometryLoader::~PhysicalGeometryLoader(){}


void
PhysicalGeometryLoader::visit(GeometryNode* geom){}

void
PhysicalGeometryLoader::visit(Node* node){}

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
		}

	}

	
}

void
PhysicalGeometryLoader::check(Node*){}



}
