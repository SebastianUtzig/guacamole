#ifndef GUA_PHYSICAL_GEOMETRY_LOADER_HPP
#define GUA_PHYSICAL_GEOMETRY_LOADER_HPP

// guacamole headers
#include <gua/guacamole.hpp>
#include <gua/physics.hpp>

namespace gua{

/**
 * This class helps to load geometries or geometry groups from a file
 *  to get objects which can be simulated with physical behavior.
 *
 */
class PhysicalGeometryLoader : public NodeVisitor{

  public:

   /**
    * Constructor.
    *
    * This constructs a PhysicalGeometryLoader.
    * \param physics    Instance of Physics world is needed for physical objects.
    */
  	PhysicalGeometryLoader(physics::Physics* physics);

   /**
    * Destructor.
    *
    * This destroys a CollisionShapeCollector.
    */
  	virtual ~PhysicalGeometryLoader();

    /**
     * Visits a GeometryNode
     *
     * This function visits a GeometryNode of a group's subgraph.
     *
     * \param geom    Pointer to GeometryNode
     */
    void                          visit(GeometryNode* geom);

    /**
     * Visits a Node
     *
     * This function visits a Node of a group's subgraph.
     *
     * \param node    Pointer to Node
     */
    void                          visit(Node* node);


    /**
     * Load geometry from file and create new nodes with physical behavior.
     *
     * Returns a PhysicalNode where physical simulations can be started by calling simulate(true).
     *
     * \param node_name           Name of loaded geometry.
     * \param file_name           Path to geometry file.
     * \param fallback_material   Material of loaded geometry.
     * \param mass                Mass of to be created physical objects.
     * \param collision_shape     CollisionShape of to be created physical objects.
     */
    std::shared_ptr<PhysicalNode> create_physical_objects_from_file
                            (std::string const& node_name,
                            std::string const& file_name,
                            std::string const& fallback_material,
                            float mass = 1.0,
                            std::shared_ptr<physics::CollisionShapeNode> const& collision_shape = nullptr
                            );



  private:

    physics::Physics* physics_;
    float             mass_;

    GeometryLoader    loader_;






};

}

#endif