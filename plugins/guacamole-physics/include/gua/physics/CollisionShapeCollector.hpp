#ifndef GUA_COLLISION_SHAPE_COLLECTOR_HPP
#define GUA_COLLISION_SHAPE_COLLECTOR_HPP

// guacamole headers
#include <gua/guacamole.hpp>


namespace gua{

class PhysicalNode;

/**
 * This class can traverse a PhysicalNode subgraph to collect or generate
 *  CollisiionShapeNodes out of geometries.
 *
 */
class CollisionShapeCollector : public NodeVisitor {
 
 public:

   /**
	* Constructor.
	*
	* This constructs a CollisionShapeCollector.
	*/
 	CollisionShapeCollector();

   /**
    * Destructor.
    *
    * This destroys a CollisionShapeCollector.
    */
 	virtual ~CollisionShapeCollector();


    /**
    * Starts a new PhysicalNode's subgraph traversal.
    * (Stored members from previous traversals will be overwritten.)
    *
    */
 	void 		check(PhysicalNode* pn);

   /**
    * Will not visit PhysicalNode's subgraph if he is already simulating. 
    *
    */
 	void 		visit(PhysicalNode* node);

  /**
   * Visits a Node
   *
   * This function visits a Node
   *
   * \param node    Pointer to Node
   */
 	void 		visit(Node* node);

   /**
   * Visits a GeometryNode
   *
   * This function visits a GeometryNode and will either collect or generate 
   *  a CollisionShapeNode representation for it if the parent is a PhysicalNode.
   * \param geom    Pointer to Node
   */
 	void		visit(GeometryNode* geom);

   /**
    * Returns the center of mass of currently collected CollisionShapeNodes.
    *  (Needs to traverse a subgraph before.)
    *
    */
 	math::vec3	get_center_of_mass()const;

   /**
    * Adds all collected shapes as children of given new RigidBodyNode
    *  and transforms them into its coordinate system.
    *
    * \param rb 	Parent of currently collected CollisionShapeNodes.
    *
    */
 	void 		add_shapes_to_rb(physics::RigidBodyNode* rb);




 private:

 	PhysicalNode* current_root_;
 	std::list<std::tuple<std::shared_ptr<physics::CollisionShapeNode>,math::mat4,float>> collected_collision_shapes_;



};


}

#endif