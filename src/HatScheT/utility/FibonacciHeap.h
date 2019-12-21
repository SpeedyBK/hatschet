/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Benjamin Lagershausen-Kessler (benjaminkessler@student.uni-kassel.de)

    Copyright (C) 2019

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

/*
 * Fibonacci Heaps are a datastructures to implement prioity queues fast and efficiently.
 * Complexity of operations:
 * -- insert_node: O(1);
 * -- get_Minimum: O(1);
 * -- extract_Minimum: O(log n);
 *
 * [1] Michael L. Fredman, Robert E. Tarjan; Fibonacci Heaps and Their Uses in Improved Network Optimization Algorithms
 * Journal of the Association for Computing Machinery 1987
 */

#ifndef HATSCHET_FIBONACCIHEAP_H
#define HATSCHET_FIBONACCIHEAP_H

#include <HatScheT/Vertex.h>

namespace HatScheT{

  /*!
   * Datastructure to store stuff in the heap.
   */
  struct node {
    //Structure
    node *parent;
    node *child;
    node *left;
    node *right;

    //Stored Value
    int key;

    //Marking
    bool is_marked;

    //Number of children a node has.
    int rank;

    //Payload:
    Vertex* v;
  };

  class FibonacciHeap {

  public:

    FibonacciHeap() = default;
    ~FibonacciHeap() = default;

    /*!
     * Function to insert nodes in the heap. In our case the Fibonacci Heap is used to store Vertices of a constraint
     * Graph and the cost of each vertex. If no "payload" is needed v can just be a nullptr.
     * @param value will be the key of the node in our heap.
     * @param v is a kind of playload attached to the node.
     */
    void insert_node(int value, Vertex *v);

    /*!
     * Extracts the minimum stored Value from the F-Heap and returns it.
     * @return Min. Stored Value.
     */
    int extract_Minimum();

    /*!
     * Same as extract_Minimum, but returns the associated Vertex as well.
     * @return
     */
    pair <int, Vertex*> extract_Minimum_with_Payload();

    /*!
     * @return Returns the minimal value stored in the heap without removing it.
     */
    int get_minimum_Key();

    /*!
     * @returns Returns the minimal value stored in the heap an the associated vertex*, without removing it.
     */
    pair<int, Vertex*> get_minimum_Key_and_Payload();

    /*!
     * @return A boolean which indicates if the Heap is empty;
     */
    bool is_empty();

    /*!
     * Function to print the heap.
     */
    void display_heap();

    /*!
     * Subfunction for display_heap
     */
    void display_childs(node* np);

  private:

    /*!
     * Setting up the Parent - Child - Relationship between nodes.
     * @param nptr2
     * @param nptr1
     */
    void pclink(node* nptr2, node* nptr1);

    /*!
     * Consolidates the heap after for example an extraction of the minimum.
     * ToDo: Fix some bugs.
     */
    void consolidate_heap ();

    /*!
     * Number of nodes in the heap.
     */
    int number_of_nodes = 0;

    /*!
     * A pointer to the minimal node.
     */
    node* min_node = nullptr;

  };
}

#endif //HATSCHET_FIBONACCIHEAP_H
