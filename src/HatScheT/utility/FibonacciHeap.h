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
 * -- insert(): O(1);
 * -- get_Minimum: O(1);
 * -- extract_Minimum: O(log n);
 *
 * [1] Michael L. Fredman, Robert E. Tarjan; Fibonacci Heaps and Their Uses in Improved Network Optimization Algorithms
 * Journal of the Association for Computing Machinery 1987
 */

#ifndef HATSCHET_FIBONACCIHEAP_H
#define HATSCHET_FIBONACCIHEAP_H

#include <cstddef>
#include <math.h>
#include <limits>
#include <iostream>


namespace HatScheT {

  template<class T, class Comp = std::less<T>>
  class FibonacciHeap {

  public:

    ///////////////////////
    /// Data Structures ///
    ///////////////////////

    /*!
     * The Fibnode-Object is the Datastructure to save stuff in the F-Heap. It contains 4 Fibnode-Pointers wich can point to the
     * Neighbor-Fibnodes. This makes it possible to establish a double circular link between the Fibnodes in the rootlist of the
     * F-Heap or in childlists.
     */
    class FibNode {
    public:
      FibNode(T k, void *pl)
          : key(std::move(k)), mark(false), parent(nullptr), left(nullptr), right(nullptr), child(nullptr), degree(-1),
            payload(pl) {
      }

      ~FibNode() = default;

      T key;
      bool mark;
      FibNode *parent;
      FibNode *left;
      FibNode *right;
      FibNode *child;
      int degree;
      void *payload;
    };


    ////////////////////////////////////
    /// Constructors and Destructors ///
    ////////////////////////////////////
    FibonacciHeap() : FibonacciHeap(std::less<T>()) {
    }

    FibonacciHeap(Comp comp) : n(0), min(nullptr), comp(comp) {
    }

    ~FibonacciHeap() { clear(); }


    ///////////////////////
    /// Member Methodes ///
    ///////////////////////

    /*!
     * Clears the heap.
     */
    void clear() {
      delete_fibnodes(min);
      min = nullptr;
      n = 0;
    }

    /*!
     * Deletes all the nodes linked to x, and all childs of nodes linked to x.
     * 1. If the heap is empty, we are already done.
     * 2. Interate through the list x is linked in and delete nodes. If a node has a child the function
     *    calls itself on the child.
     * @param x
     */
    void delete_fibnodes(FibNode *x) {
      // 1
      if (x == nullptr) {
        return;
      }

      FibNode *cur = x;

      // 2
      while (true) {
        /*std::cerr << "cur: " << cur << std::endl;
          std::cerr << "x: " << x << std::endl;*/
        if ((cur->left != nullptr) && cur->left != x) {
          //std::cerr << "cur left: " << cur->left << std::endl;
          FibNode *tmp = cur;
          cur = cur->left;
          if (tmp->child)
            delete_fibnodes(tmp->child);
          delete tmp;
        } else {
          if (cur->child)
            delete_fibnodes(cur->child);
          delete cur;
          break;
        }
      }
    }

    /*!
     * Methode to insert the FibNode X to the heap. By doing the following steps:
     * 1. x.degree = 0, x.parent = nullptr, x.child = nullptr, x.mark = false;
     * 2. If the Heap is empty, create a new rootlist with just x and set the min-ptr to x;
     * 3. If the Heap is not empty, x is added to the rootlist left of the min-ptr.
     *    If x.key < min.key the min-ptr has to be updated.
     * 4. Increment number of nodes (n) in the heap.
     * @param x
     */
    void insert(FibNode *x) {
      // 1
      x->degree = 0;
      x->parent = nullptr;
      x->child = nullptr;
      x->mark = false;
      // 2
      if (min == nullptr) {
        // Since x is all alone :( , it can just be linked to itself to establisch the double-circular linking.
        min = x->left = x->right = x;
      }
      // 3
      else {
        min->left->right = x;
        x->left = min->left;
        min->left = x;
        x->right = min;
        if (comp(x->key, min->key)) {
          min = x;
        }
      }
      // 4
      ++n;
    }

    /*!
     * Return the minimum node of the heap without removing it.
     */
    FibNode *get_minimum() { return min; }

    /*!
     * extract_min() returns the minimum node of the heap and removes it from the heap.
     * 1. Save a copy (z) of the min-ptr, and check if the heap is empty.
     * 2. If z has a child (x)
     * 3.   For each child x of node z
     * 4.     Add x to the rootlist of the heap. And remove the parent-ptr
     * 5.   Remove z from the rootlist.
     * 6.   If z.right points to z
     * 7.     z is the last node in the heap, so the min_ptr has to be set as nullptr.
     * -.   Else
     * 8.     The minpointer has to be set to the right of z. And the Heap needs to be consolidated.
     * 9.   The number of nodes in the Heap has to be decremented.
     * @return z
     */
    FibNode *extract_min() {
      FibNode *z, *x, *next;
      FibNode **childList;

      // 1
      z = min;
      if (z != nullptr) {
        // 2
        x = z->child;
        if (x != nullptr) {
          // 3
          childList = new FibNode *[z->degree];
          next = x;
          for (int i = 0; i < (int) z->degree; i++) {
            childList[i] = next;
            next = next->right;
          }
          for (int i = 0; i < (int) z->degree; i++) {
            x = childList[i];
            // 4
            min->left->right = x;
            x->left = min->left;
            min->left = x;
            x->right = min;
            x->parent = nullptr;
          }
          delete[] childList;
        }
        // 5
        z->left->right = z->right;
        z->right->left = z->left;
        // 6
        if (z == z->right) {
          // 7
          min = nullptr;
        } else {
          // 8
          min = z->right;
          consolidate();
        }
        // 9
        n--;
      }
      return z;
    }


    /*!
     * After removing a node from the heap, the heap needs to be consolidated as long as it is not empty.
     * At first the maximum degree a node can have is calculated.
     * 1. An Array of nodeptrs is created and initialized with nullptrs.
     * 2. For each node w in the rootlist of the heap do.
     *    x = w
     *    d = x.degree
     * 3. While (A[d] != nullptr) [This condition is false untill we find 2 rootnodes with the same degree.]
     *    y = A[d]
     *    Then the key of x is compared with the key of y. If y -> key < x. -> key,
     * 4.   x and y are swapped.
     * 5. Now we link the note with the greater key as a child of the other node. This increases the degree of the
     *    parent node and removes the tree which now is a child of the other node.
     * 6. So we can set A[d] now to nullptr, since A[d] now either has an incremented degree, or is a child of the other node.
     *    Since we now a created a tree with a greater degree, we have to check again is we have found a tree with that
     *    degree earlier. If that is the case, do the do the while loop again.
     * 7. x is added to the Array.
     * 8. If those steps are done, the structure of the heap is updated. Now we just have to update the minptr and the rootlist
     * 9. Loop through the A Array.
     * 10.If min == nullptr
     *      then we create a new rootlist with just A[i]
     *    else
     *       A[i] has to be added to the rootlist.
     *       min.key and A[i].key are compared and the minptr is set to the node with the smaller key.
     */
    void consolidate() {
      FibNode *w, *next, *x, *y, *temp;
      FibNode **A, **rootList;
      // Max degree <= log base golden ratio of n
      int d, rootSize;
      int max_degree = static_cast<int>(floor(
          log(static_cast<double>(n)) / log(static_cast<double>(1 + sqrt(static_cast<double>(5))) / 2)));

      // 1
      A = new FibNode *[max_degree + 2]; // plus two both for indexing to max degree and so A[max_degree+1] == NIL
      std::fill_n(A, max_degree + 2, nullptr);
      // 2
      w = min;
      rootSize = 0;
      next = w;
      do {
        rootSize++;
        next = next->right;
      } while (next != w);
      rootList = new FibNode *[rootSize];
      for (int i = 0; i < rootSize; i++) {
        rootList[i] = next;
        next = next->right;
      }
      for (int i = 0; i < rootSize; i++) {
        w = rootList[i];
        x = w;
        d = x->degree;
        // 3
        while (A[d] != nullptr) {
          y = A[d];
          if (comp(y->key, x->key)) {
            // 4
            temp = x;
            x = y;
            y = temp;
          }
          // 5
          fib_heap_link(y, x);
          // 6
          A[d] = nullptr;
          d++;
        }
        // 7
        A[d] = x;
      }
      delete[] rootList;

      // 8
      min = nullptr;
      // 9
      for (int i = 0; i < max_degree + 2; i++) {
        if (A[i] != nullptr) {
          // 10
          if (min == nullptr) {
            min = A[i]->left = A[i]->right = A[i];
          } else {
            min->left->right = A[i];
            A[i]->left = min->left;
            min->left = A[i];
            A[i]->right = min;
            // 22
            if (comp(A[i]->key, min->key)) {
              min = A[i];
            }
          }
        }
      }
      delete[] A;
    }

    /*!
     * fib_heap_link(y,x) links y as a child of x
     * 1. remove y from the root list of heap
     * 2. make y a child of x, increment x.degree
     * 3. mark y as FALSE.
     */
    void fib_heap_link(FibNode *y, FibNode *x) {
      // 1
      y->left->right = y->right;
      y->right->left = y->left;
      // 2
      if (x->child != nullptr) {
        x->child->left->right = y;
        y->left = x->child->left;
        x->child->left = y;
        y->right = x->child;
      } else {
        x->child = y;
        y->right = y;
        y->left = y;
      }
      y->parent = x;
      x->degree++;
      // 3
      y->mark = false;
    }

    /*!
     * union_FibonacciHeap(H1,H2) combines two heap H1 and H2 to one heap H and returns it.
     * 1. A new heap H is created and the minptr of H is set to the minptr of H1.
     * 2. concatenate the root list of H2 with the root list of H
     * 3. if (H1.min == nullptr) or (H2.min != nullptr and H2.min.key < H1.min.key), means if H1 is empty or H2.min.key is smaller than H1.min.key
     *  	  The minptr of the combined heap is set the minptr of H2.
     * 5. The number of nodes of H is the sum of the nodes of H1 and H2.
     * 7. @return H
     */

    static FibonacciHeap *union_FibonacciHeap(FibonacciHeap *H1, FibonacciHeap *H2) {
      // 1
      auto *H = new FibonacciHeap();
      H->min = H1->min;
      // 2
      if (H->min != nullptr && H2->min != nullptr) {
        H->min->right->left = H2->min->left;
        H2->min->left->right = H->min->right;
        H->min->right = H2->min;
        H2->min->left = H->min;
      }
      // 3
      if (H1->min == nullptr || (H2->min != nullptr && H1->comp(H2->min->key, H1->min->key))) {
        H->min = H2->min;
      }
      // 6
      H->n = H1->n + H2->n;
      // 7
      return H;
    }

    /*!
     * decrease_key(x,k) can decrease the key of the node x.
     * Decreasing the key of a node can result in a violation of the rule that the parent node of x must always
     * have a smaller key than x so decrease_key has to take care of that.
     * 1. if k > x.key the key of x can't be decreased and we can simply return.
     * 2. The key of x is set to k
     * 3. y is the parent_node of X
     * 4. If y exists and x.key is smaller than y.key
     * 5.   Cut x from the childlist of y and insert x in the rootlist of the heap.
     * 6. 	CASCADING-CUT(H,y)
     * 7. if the key of y is smaller than the minkey
     * 8. 	update H.min to x
     */
    void decrease_key(FibNode *x, T k) {
      FibNode *y;

      // 1
      if (comp(x->key, k)) {
        //throw HatScheT::Exception("Fibonacci Heap decrease_key(): New key is greater than current key" );
        return;
      }
      // 2
      x->key = std::move(k);
      // 3
      y = x->parent;
      // 4
      if (y != nullptr && comp(x->key, y->key)) {
        // 5
        cut(x, y);
        // 6
        cascading_cut(y);
      }
      // 7
      if (comp(x->key, min->key)) {
        // 8
        min = x;
      }
    }

    /*!
     * cut(x,y) removes the node x from the childlist of y and adds x to the rootlist of the heap.
     * 1. remove x from the child list of y, decrement y.degree
     * 2. add x to the root list of H
     * 3. set the parent of x to nullptr
     * 4. mark x as false
     */
    void cut(FibNode *x, FibNode *y) {
      // 1
      if (x->right == x) {
        y->child = nullptr;
      } else {
        x->right->left = x->left;
        x->left->right = x->right;
        if (y->child == x) {
          y->child = x->right;
        }
      }
      y->degree--;
      // 2
      min->right->left = x;
      x->right = min->right;
      min->right = x;
      x->left = min;
      // 3
      x->p = nullptr;
      // 4
      x->mark = false;
    }

   /*!
    * cascading_cut(y)
    * 1. z is the Parentnode of y
    * 2. 	if y is not marked
    * 3. 		mark y
    * 4. 	else
    *       CUT(H,y,z)
    * 5. 		CASCADING-CUT(H,z)
    */
    void cascading_cut(FibNode *y) {
      FibNode *z;

      // 1
      z = y->parent;
      // 2
      if (z != nullptr) {
        // 3
        if (!y->mark) {
          // 4
          y->mark = true;
        } else {
          // 5
          cut(y, z);
          // 6
          cascading_cut(z);
        }
      }
    }

    /*!
     * remove_fibnode(x) sets to the minimum so that it hits the top of the heap, then easily remove.
     */
    void remove_fibnode(FibNode *x) {
      decrease_key(x, std::numeric_limits<T>::min());
      FibNode *fn = extract_min();
      delete fn;
    }

    /*!
     * mapping operations to STL-compatible signatures.
     */
    bool empty() const {
      return n == 0;
    }

    FibNode *topNode() {
      return get_minimum();
    }

    T &top() {
      return get_minimum()->key;
    }

    void pop() {
      if (empty())
        return;
      FibNode *x = extract_min();
      if (x)
        delete x;
    }

    T get_extract_min(){
      T temp = top();
      pop();
      return temp;
    }

    FibNode *push(T k, void *pl) {
      auto *x = new FibNode(std::move(k), pl);
      insert(x);
      return x;
    }

    FibNode *push(T k) {
      return push(std::move(k), nullptr);
    }

    unsigned int size() {
      return (unsigned int) n;
    }

    int n;
    FibNode *min;
    Comp comp;

  };
}

#endif //HATSCHET_FIBONACCIHEAP_H
