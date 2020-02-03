//
// Created by bkessler on 14/01/20.
// * [1] G.Ramalingam, J. Song, L. Joskowicz, R.E. Miller; Solving Systems of Difference Constraints Incrementally;
// * Algorithmica 1999
//
//

#ifndef HATSCHET_SDCSOLVER_H
#define HATSCHET_SDCSOLVER_H

#include <HatScheT/Graph.h>
#include "FibonacciHeap.h"

namespace HatScheT {

  struct SDCConstraint{
    Vertex* VSrc;
    Vertex* VDst;
    int constraint;
  };

  class SDCSolver {
  private:

    ///////////////////////
    /// Data Structures ///
    ///////////////////////

    /*!
     * To solve SDC-Problems incrementaly we need to be able to modify graphs. For Example add and remove constraints.
     * So ConstraintGraph is derived from Graph and the needed functionality is added.
     */
    class ConstraintGraph : public Graph {

    public:

      /*!
       * Function to remove an edge from ConstraintGraph. Iterates over the Edges of the Constraint-Graph and checks
       * if there is an Edge between srcVertex and dstVertex and removes it.
       * @param srcVertex Source vertex of the edge to delete.
       * @param dstVertex Destination vertex of the egde to delete.
       */
      void removeEdge(Vertex &srcVertex, Vertex &dstVertex);

      /*!
       Looks up if an egde exist between src and dst.
       * @param src Source vertex of the edge.
       * @param dst Destination vertex of the edge.
       * @return True if edge exists, otherwise false.
       */
      bool doesEdgeExistID(Vertex *src, Vertex *dst);

      /*!
       * Creates an Edge in the constraint graph.
       * @param Vsrc First Variable of the constraint.
       * @param Vdst Second Variable of the constraint.
       * @param distance Right side of the constraint.
       * @param dependencyType not used.
       * @return Returns the Constraintedge.
       */
      Edge& createEdgeSDC (Vertex &Vsrc, Vertex &Vdst, int distance, Edge::DependencyType dependencyType);

      /*!
       * Creates a vertex, assigns a non-existing id and inserts it in the graph G and in the index of vertices.
       * \return The vertex
       */
      Vertex& createVertexSDC();

      /*!
       * Creates a vertex, inserts it in the graph G in the Index of Vertices.
       * \param The id of the vertex
       * \return The vertex
       */
      Vertex& createVertexSDC(int id);

      /*!
       * Uses the map "vertex_index" to return the vertex with id you looked for.
       * @param id of the Vertex you look for.
       * @return the Vertex with ID id.
       */
      Vertex& getVertexbyIdSDC(int id);

      /////////////////
      /// Variables ///
      /////////////////

      map <int, Vertex*> vertex_index;
    };

    /*!
     * Funtion so set the source vertex for the single source shortest path problem.
     * And creates edges from the start_vertex to all other vertices.
     */
    void set_start_vertex();

    /*!
     * Since the SDC-Solver uses a Fibonacci Heap to implement Dijkstras Algorithm, this function is used to get
     * stuff back from this heap and putting it in a pair <Vertex*, int>
     * @param H Reference to the Fibonacci Heap used in the solver.
     * @return Returns the Vertex and the saved costs for this Vertex.
     */
    pair <Vertex*, int> fetch_from_heap(FibonacciHeap<int> &H);

    /*!
     * If v is in the Heap, it sets key of v to k. If v is not in the heap, it inserts v with key k into the heap.
     * @param H: Fibonacci Heap.
     * @param v: Vertex which key should be changed.
     * @param k: Value the key of v is changed to.
     */
    void adjust_Heap(FibonacciHeap<int> &H, Vertex* v, int k);

    /*!
     * If v is in the Heap the funtion returns the key of v, else it returns INT_MAX;
     * @param H: Fibonacci Heap.
     * @param v: Vertex which key should be changed.
     * @return if v is in the heap the key of v. If v is not in the heap it returns INT_MAX.
     */
    int key_of(FibonacciHeap<int> &H, Vertex *v);

    /*!
     * Graphbased Representation of the SDC-System.
     */
    ConstraintGraph cg;

    /*!
     * Status off the solver.
     * 0  = Ready;
     * 10 = Inital Solution computed, feasible.
     * 11 = Inital Solution computed, infeasible.
     * 20 = System feasible after adding a constraint.
     * 21 = System infeasible after adding a constraint.
     * .. = ...
     */
    int solver_status;

    /*!
     * Solution for the given SDC-System.
     */
    map <Vertex*, int> solution;

    /*!
     * Sartvertex to solve the single-source-shortest-path problem.
     */
    Vertex* startvertex;

  public:

    SDCSolver();
    ~SDCSolver() = default;

    /*!
     * Adds SDC-Constraints to the solver. If more than one constraint with equal variables like A - B < 2 and A - B < 4
     * the constraint with the greater value is ignored since we a looking for a shortest Path.
     */
    void add_sdc_constraint (SDCConstraint constr);

    /*!
     * Removes SDC-Constraints from the solver. Prints out a message if a non existing constraint should be removed.
     * @param Vsrc Second Variable of the constraint.
     * @param Vdst First Variable of the constraint.
     */
    void remove_sdc_constraint (Vertex &Vsrc, Vertex &Vdst);

    /*!
     * Prints the constraint graph.
     */
    void print_Constraint_Graph(){cout << this->cg << endl;};

    /*!
     * Function to check the status of the solver.
     * @return solver_status.
     */
    int get_solver_status(){ return solver_status; };

    /*!
     * This computes an initial solution of the SDC-System by applying the Bellman-Ford-Algorithm on the system and checks
     * if the system is feasible.
     */
    void compute_inital_solution();

    /*!
     * This Function allows to pass a solution of an SDC-System into the solver, which can be used to start an incremental solving.
     * @param known_solution Known Solution for an existing SDC-System.
     */
    void set_initial_solution(map <Vertex*, int> &known_solution);

    /*!
     * This function returns the computed solution if the system is feasible. If the system is infeasible, it will
     * throw an error.
     * @return The solution of the SDC-System.
     */
    map <Vertex*, int> get_solution();

    /*!
     * Funtction to create SDC-Constraints. Like A - B < 5
     * @param src Second Variable of the constraint (B)
     * @param dst First Variable of the contraint (A)
     * @param c Right side of the constraint (5)
     * @return a Struct which represents the Constraint.
     */
    SDCConstraint create_sdc_constraint(Vertex* src, Vertex* dst, int c);

    /*!
     * @param constr A new constraint which should be added to a feasible system of singe difference constraints.
     * This funtion is used to add additional constraints to a system of single difference constraints and checks
     * if thereafter the system is still feasible and if it is the funtions returns a solution for the new system.
     * Essentially Dijkstras Algorithm with an Fibonacci Heap as priority queue and a path length transformation
     * is used. [1]
     *
     * pseudocode:
     * 1.  add edge u->v
     * 2.  length(u->v) = c
     * 3.  D' = D (D is last feasible solution)
     * 4.  init empty priority queue (H)
     * 5.  insert v to H with key = 0;
     * 6.  while (priority queue != empty) do
     * 7.    (x, dist_x) := FindAndDeleteMin(H)
     * 8.    if (D(u) + length(u->v) + (D(x) + dist_x - D(v)) < D(x)) then
     * 9.       if (x = u ) then
     *            The system is infeasible!
     * 10.        remove edge u->v
     * 11.        return infeasible
     * 12.      else
     * 13.        D'(x) = D(u) + length(u->v) + (D(x) + dist_x - D(v))
     * 14.        for each vertex y in Succ (x) do
     * 15.           scaledPathLength = dist_x + ( D(x) + length(x->y) - D(y))
     * 16.           if (scaledPathLength < KeyOf(H, y))
     * 17.               AdjustHeap(H, y, scaledPathLength)
     * 18.           end if
     * 19.        end for
     * 20.      end if
     * 21.   end if
     * 22. end while
     *     The system is feasible!
     * 23. D = D'
     * 24. return feasible
     */
    void add_to_feasible(SDCConstraint constr);

  };

}


#endif //HATSCHET_SDCSOLVER_H
