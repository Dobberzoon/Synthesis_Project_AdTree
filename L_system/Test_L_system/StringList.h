//
// Created by hyde on 5/30/21.
//

#ifndef L_SYSTEM_STRINGLIST_H
#define L_SYSTEM_STRINGLIST_H

#endif //L_SYSTEM_STRINGLIST_H

#include <iostream>
#include <string>


class StringList{
public:
    struct LstrNode{
        std::string Lstr;
//        size_t root_;
        LstrNode *next;
        LstrNode(const std::string str, LstrNode *p = nullptr) {
//            root_ = r;
            Lstr = str;
            next = p;
        }
        LstrNode(LstrNode *p=nullptr) {
            next = p;
        }
    };
    StringList();
    ~StringList();

    void clear();

    bool empty() const {return head -> next == nullptr;}

    int size() const {return curLength;}
    LstrNode* get_head() const {return head;}

    void traverse();

//    void print();

    std::string get_ls();

    // TODO:
    LstrNode* insert(LstrNode* p, const std::string &var);

//    void remove(LstrNode* p);

//    int search(const std::string &var) const;

//    int prior(const std::string &var) const;




//    LstrNode *getPostion(int i) const;
private:


    LstrNode *head;
    LstrNode *tail;
    int curLength;

};


StringList::StringList() {
    head = tail = new LstrNode();
    curLength = 0;
}

StringList::~StringList() {
    clear();
    delete head;
}

void StringList::clear() {
    LstrNode *p, *tmp;
    p = head -> next;
    while (p!= nullptr){
        tmp = p;
        p= p->next;
        delete tmp;
    }
    head -> next = nullptr;
    tail = head;
    curLength = 0;
}

void StringList::traverse() {
    LstrNode *p = head->next;
    std::cout << "traverse: ";
    while (p!= nullptr){
        std::cout << p->Lstr;
        p=p->next;
    }
}

std::string StringList::get_ls() {
    std::string ls;
    LstrNode *p = head->next;
    while (p!= nullptr){
        ls+=p->Lstr;
    }
    return ls;
}

StringList::LstrNode* StringList::insert(LstrNode *p, const std::string &var) {
//    if (p->Lstr.empty() && p->next== nullptr) {
//        std::cout << "insert out of range." << std::endl;
//        return;
//    }
//    else if ()
    LstrNode *q;
    if (p->next!= nullptr) q = new LstrNode(var, p->next);
    else {
        q = new LstrNode(var);
        tail = q;
    }
    p->next = q;
    curLength ++;
    return q;
}

//typename StringList::LstrNode *StringList::getPostion(int i) const {
//    if (i < -1 || i > curLength-1) return nullptr;
//    LstrNode *p = head;
//    int count = 0;
//    while (count <= i){
//        p = p->next;
//        count++;
//    }
//    return p;
//}

