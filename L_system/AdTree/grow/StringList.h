//
// Created by hyde on 6/3/21.
//

#ifndef L_SYSTEM_STRINGLIST_H
#define L_SYSTEM_STRINGLIST_H


class StringList {
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
    int get_len() const {return curLength;}
    void printLstr();

    std::string get_ls();
    LstrNode* insert(LstrNode* p, const std::string &var);

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

void StringList::printLstr() {
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

#endif //L_SYSTEM_STRINGLIST_H
