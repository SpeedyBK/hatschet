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

#include <malloc.h>
#include <cmath>
#include "FibonacciHeap.h"

namespace HatScheT {


  //////////////////
  ///Insert Nodes///
  //////////////////
  void HatScheT::FibonacciHeap::insert_node(int value, HatScheT::Vertex *v) {

    node *new_node = (node *) malloc(sizeof(node));

    new_node->key = value;
    new_node->v = v;
    new_node->parent = nullptr;
    new_node->child = nullptr;
    new_node->left = new_node;
    new_node->right = new_node;
    new_node->is_marked = false;
    new_node->rank = 0;

    this->number_of_nodes++;

    if (min_node != nullptr) {
      (min_node->left)->right = new_node;
      new_node->right = this->min_node;
      new_node->left = this->min_node->left;
      min_node->left = new_node;
      if (new_node->key < this->min_node->key) {
        this->min_node = new_node;
      }
    } else {
      min_node = new_node;
    }
  }

  ///////////////////////////////////////////////
  ///Inititalisation of Parent - Child - Links///
  ///////////////////////////////////////////////
  void FibonacciHeap::pclink(node *nptr2, node *nptr1) {

    (nptr2->left)->right = nptr2->right;
    (nptr2->right)->left = nptr2->left;

    if (nptr1->right == nptr1){
      min_node = nptr1;
    }

    nptr2->left = nptr2;
    nptr2->right = nptr2;
    nptr2->parent = nptr1;

    if (nptr1->child == nullptr){
      nptr1->child = nptr2;
    }

    nptr2->right = nptr1->child;
    nptr2->left = (nptr1->child)->left;
    ((nptr1->child)->left)->right = nptr2;
    (nptr1->child)->left = nptr2;
    if (nptr2->key < (nptr1->child)->key) {
      nptr1->child = nptr2;
    }
    nptr1->rank++;

  }

  ////////////////////////////
  ///Consolidating the heap///
  ////////////////////////////
  void FibonacciHeap::consolidate_heap() {

    int temp1;

    //Calculate the "max.rank" possible in the heap. And setup an array with "max.rank" elements.
    int max_rank = ceil(log(number_of_nodes)/log(2));
    //Set up an array of pointers with size ciel(max_rank);
    node* rankarr[max_rank];
    //Initialize the Array with nullptrs.
    for (int i = 0; i <= max_rank; i++){
      rankarr[i] = nullptr;
    }
    //Set up 4 temp pointers;
    node* ptr1 = min_node;
    node* ptr2;
    node* ptr3;
    node* ptr4 = ptr1;

    //Loop through the root list oft the heap. And check the ranks of all the trees. When this process is done, only
    //only trees with unequal rank can exist in the heap.
    do{
      temp1 = ptr1->rank;
      ptr4 = ptr4->right;

      //If 2 trees with the same rank are found. We put the tree with the higher key stored in the root node
      //in a child relationship to the other tree.
      while (rankarr[temp1] != nullptr){
        ptr2 = rankarr[temp1];
        if (ptr1->key > ptr2->key){
          ptr3 = ptr1;
          ptr1 = ptr2;
          ptr2 = ptr3;
        }
        //Kinda updating the min_node.
        if (ptr2 == min_node){
          min_node = ptr1;
        }
        //Parent Child Linking
        pclink(ptr2, ptr1);
        if (ptr1->right == ptr1){
          min_node = ptr1;
        }
        rankarr[temp1] = nullptr;
        temp1++;
      }
      rankarr[temp1] = ptr1;
      ptr1 = ptr1->right;
    }while (ptr1 != min_node);


    /* ToDo: seems like this whole for-loop is BS!!!
    for (int j = 0; j <= max_rank; j++) {
      if (rankarr[j] != nullptr) {
        rankarr[j]->left = rankarr[j];
        rankarr[j]->right = rankarr[j];
        if (min_node != nullptr) {
          (min_node->left)->right = rankarr[j];
          rankarr[j]->right = min_node;
          rankarr[j]->left = min_node->left;
          min_node->left = rankarr[j];
          if (rankarr[j]->key < min_node->key) {
            min_node = rankarr[j];
          }
        }
        else {
          min_node = rankarr[j];
        }
        if (min_node == nullptr)
          min_node = rankarr[j];
        else if (rankarr[j]->key < min_node->key)
          min_node = rankarr[j];
      }
    }
    */
  }

  /////////////////////
  ///Extract Minimum///
  /////////////////////
  int FibonacciHeap::extract_Minimum() {

    int min_val = min_node->key;

    if (this->min_node == nullptr){
      cout << "Heap is empty" << endl;
    }else{
      node* temp = min_node;
      node* ptr = temp;
      node* x = nullptr;
      if (temp->child != nullptr){
        x = temp->child;
        do {
          ptr = x->right;
          (min_node->left)->right = x;
          x->right = min_node;
          x->left = min_node->left;
          min_node->left = x;
          x->parent = nullptr;
          x = ptr;
        }while (ptr != temp->child);
      }
      (temp->left)->right = temp->right;
      (temp->right)->left = temp->left;
      min_node = temp->right;
      if (temp == temp->right && temp->child == nullptr)
        min_node = nullptr;
      else {
        min_node = temp->right;
        consolidate_heap();
      }
      number_of_nodes--;
    }
    return min_val;
  }


  pair<int, Vertex *> FibonacciHeap::extract_Minimum_with_Payload() {

    pair<int, Vertex *> min_pair = make_pair(min_node->key, min_node->v);

    if (this->min_node == nullptr){
      cout << "Heap is empty" << endl;
    }else{
      node* temp = min_node;
      node* ptr = temp;
      node* x = nullptr;
      if (temp->child != nullptr){
        x = temp->child;
        do {
          ptr = x->right;
          (min_node->left)->right = x;
          x->right = min_node;
          x->left = min_node->left;
          min_node->left = x;
          if (x->key < min_node->key){
            min_node = x;
          }
          x->parent = nullptr;
          x = ptr;
        }while (ptr != temp->child);
      }
      (temp->left)->right = temp->right;
      (temp->right)->left = temp->left;
      min_node = temp->right;
      if (temp == temp->right && temp->child == nullptr)
        min_node = nullptr;
      else {
        min_node = temp->right;
        consolidate_heap();
      }
      number_of_nodes--;
    }
    return min_pair;
  }

  ////////////////////
  ///Check if empty///
  ////////////////////
  bool FibonacciHeap::is_empty() {
    if (min_node == nullptr){
      return true;
    }else{
      return false;
    }
  }

  ////////////////////////
  ///Get Minimum Values///
  ////////////////////////
  int FibonacciHeap::get_minimum_Key() {
    return this->min_node->key;
  }

  pair<int, Vertex*> FibonacciHeap::get_minimum_Key_and_Payload() {
    return make_pair(this->min_node->key, this->min_node->v);
  }


  //////////////////
  ///Display Heap///
  //////////////////
  void FibonacciHeap::display_heap() {

    node *ptr = min_node;
    if (ptr == nullptr) {
      cout << "The Heap is Empty" << endl;
    } else {
      cout << "The root nodes of the heap are: " << endl;

      do {
        display_childs(ptr);
        ptr = ptr->right;
        if (ptr != min_node) {
          cout << "  -->  ";
        }
      } while (ptr != min_node && ptr->right != nullptr);

      cout << endl << "The heap has " << number_of_nodes << " nodes." << endl;
      cout << endl << "The minimum value stored is: " << get_minimum_Key() << endl;
    }
  }

  void FibonacciHeap::display_childs(node* np) {
    cout << np->key;
    if (np->rank != 0) {
      cout << " { ";
      display_childs(np->child);
      cout << " } ";
    }
  }
}

