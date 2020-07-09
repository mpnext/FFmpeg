#include "stdlib.h"
#include "stdio.h"
#include "string.h"

#ifndef GRAPH_STACK
#define MAX_STACK_SIZE 10000

typedef struct gStack {
    int top;
    int buffer[MAX_STACK_SIZE];
} gStack;

void gstack_pop(gStack* s);
int gstack_empty(gStack s);
int gstack_full(gStack s);
void gstack_init(gStack* s);
int gstack_top(const gStack* s);
void gstack_push(gStack* s, int value);

void gstack_pop(gStack* s) { --s->top; }
int gstack_empty(gStack s) { return s.top == -1; }
int gstack_full(gStack s) { return s.top == MAX_STACK_SIZE; }
void gstack_init(gStack* s) {
    s->top = -1;
}

int gstack_top(const gStack* s) {
    return s->buffer[s->top];
}

void gstack_push(gStack*s, int value) {
    ++s->top;
    s->buffer[s->top] = value;
}
#endif // !GRAPH_STACK

#ifndef GRAPH_ALGO
#define MAX_VERTEX_NUMBER 1000
typedef struct AdjListNode {
    int v;
    int w;
} AdjListNode;

typedef struct Graph {
    int V;
    AdjListNode adj[MAX_VERTEX_NUMBER][MAX_VERTEX_NUMBER];
    int tails[MAX_VERTEX_NUMBER];
} Graph;

Graph construct_graph(int V);
int destroy_graph(Graph* g);
void graph_add_edge(Graph* g, int u, int v, int weight);
int longestPath(Graph g, int source);
#endif // !GRAPH_ALGO

#define NINF -10000

static int adj_list_alloc(AdjListNode adj[MAX_VERTEX_NUMBER][MAX_VERTEX_NUMBER], int V) {
    int i;
    for (i = 0; i < V; ++i) {
        adj[i][0].v = -1;
        adj[i][0].w = -1;
    }
    return 0;
}

static void adj_list_free(AdjListNode adj[MAX_VERTEX_NUMBER][MAX_VERTEX_NUMBER], int V) {
    memset(adj, 0, sizeof(adj));
    int i;
    for (i = 0; i < V; ++i) {
        adj[i][0].v = -1;
        adj[i][0].w = -1;
    }
}

Graph construct_graph(int V) {
    Graph g;
    int i;
    g.V = V;
    memset(g.adj, 0, sizeof(g.adj));
    if (adj_list_alloc(g.adj, V) != 0) {
        printf("Failed to alloc memory for Linked List.");
        exit(1);
    }
    for (i = 0; i < V; ++i) {
        g.tails[i] = 0;
    }
    return g;
}

int destroy_graph(Graph* g) {
    int i = 0;
    adj_list_free(g->adj, g->V);
    g->V = -1;
    memset(g->tails, 0, sizeof(g->tails));
    return 0;
}

void graph_add_edge(Graph* g, int u, int v, int weight) {
    AdjListNode node;
    node.v = v;
    node.w = weight;
    g->adj[u][g->tails[u]] = node;
    g->adj[u][g->tails[u] + 1].v = -1;
    ++g->tails[u];
}

static void topsort(Graph* g, int v, int visited[], gStack* stack) {
    visited[v] = 1;
    int i = 0;
    for (i = 0; i < g->tails[v]; ++i) {
        AdjListNode node = g->adj[v][i];
        if (visited[node.v] == 0) {
            topsort(g, node.v, visited, stack);
        }
    }
    //printf("%d;", v);
    gstack_push(stack, v);
}

int longestPath(Graph g, int source) {
    int visited[MAX_VERTEX_NUMBER];
    int dist[MAX_VERTEX_NUMBER];
    int u;
    int i;
    AdjListNode inode;
    memset(visited, 0, sizeof(visited));
    memset(dist, 0, sizeof(dist));
    for (i = 0; i < g.V; ++i) {
        dist[i] = NINF;
    }
    dist[source] = 0;
    gStack stack;
    gstack_init(&stack);
    topsort(&g, 1, visited, &stack);

    //process the longest path in topological order
    while (!gstack_empty(stack)) {
        int u = gstack_top(&stack);

        gstack_pop(&stack);
        if (dist[u] != NINF) {
            for (i = 0; i < g.tails[u]; ++i) {
                inode = g.adj[u][i];
                if (dist[inode.v] < dist[u] + inode.w) {
                    dist[inode.v] = dist[u] + inode.w;
                }
            }
        }
    }

    for (i = 0; i < g.V; ++i) {
        printf("%d ", dist[i]);
    }
    return 0;
}
