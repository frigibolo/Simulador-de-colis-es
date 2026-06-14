#include <stdio.h> 
#include <math.h>
#include <stdlib.h>
#include <SDL3/SDL.h>

//=========================================================================================================

#define HZ 60 //Framerate
#define OBJECTS "teste1.txt" //Nome do arquivo texto utilizado para adicionar os objetos
#define E 1 //Coeficiente de restituicao
#define SCREEN_W 1000 //Largura da tela do simulador
#define SCREEN_H 600 //Altura da tela do simulador

//=========================================================================================================

typedef struct
{
    float x;
    float y;
}Vector2;


typedef struct
{
    Vector2 posi;
    Vector2 vlct;
    float mass;
    float size;
} Object;

typedef struct
{
    Object* storage;
    int* interactions;
    int count;
}World;

//=========================================================================================================

//Structures

float dot(Vector2 vec1, Vector2 vec2);
Vector2 vecProj(Vector2 vecProjected, Vector2 vecProjector);
float vecModule(Vector2 vector);
void addVelocity(Object *part, Vector2 vlct);
void collisionFisics(Object *part1, Object *part2, float e);
float timeCorrection(Object *part1, Object *part2);
void applyCollision(World *worldstr);
int getCollisions(World *worldstr);
void collisionOnWall(World *worldstr);
World startStorage();
int arrayOfObjects(Object *parts, World *worldstr, int size);
void createAnObject(Object part, World *worldstr);
Object* readText(int *size);
void drawCircle(SDL_Renderer *renderer, Object *part);

//=========================================================================================================

//Funcoes matematicas

float dot(Vector2 vec1, Vector2 vec2)
{
    float dotMult = 0.0;
    dotMult += vec1.x * vec2.x;
    dotMult += vec1.y * vec2.y;
    return dotMult;
}

Vector2 vecProj(Vector2 vecProjected, Vector2 vecProjector)
{
    Vector2 newVec;
    float mult;
    mult = (dot(vecProjected, vecProjector)) / dot(vecProjector, vecProjector);
    
    newVec.x = mult * vecProjector.x;
    newVec.y = mult * vecProjector.y;
    return newVec;
}

float vecModule(Vector2 vector)
{
    return (sqrtf((vector.x) * (vector.x) + (vector.y) * (vector.y)));
}

//=========================================================================================================

//Funcoes para a fisica

void addVelocity(Object *part, Vector2 vlct)
{
    part->vlct.x += vlct.x;
    part->vlct.y += vlct.y; 
}

void collisionFisics(Object *part1, Object *part2, float e)
{
    Vector2 vecAct = {(part1->posi.x - part2->posi.x), (part1->posi.y - part2->posi.y)};
    float v1, v2, nv1, nv2, m1, m2, dist;
    dist = vecModule(vecAct);
    if (dist == 0.0f)
        dist = 0.1f;

    vecAct.x = vecAct.x / dist;
    vecAct.y = vecAct.y / dist;

    v1 = dot(part1->vlct, vecAct);
    v2 = dot(part2->vlct, vecAct);

    if (v1 - v2 >= -0.1)
        return;

    m1 = part1->mass;
    m2 = part2->mass;

    nv1 = ((m1 * v1) + (m2 * v2) - m2 * e * (v1 - v2)) / (m1 + m2);
    nv2 = ((m1 * v1) + (m2 * v2) + m1 * e * (v1 - v2)) / (m1 + m2);

    part1->vlct.x += (nv1 - v1) * vecAct.x;
    part1->vlct.y += (nv1 - v1) * vecAct.y;

    part2->vlct.x += (nv2 - v2) * vecAct.x;
    part2->vlct.y += (nv2 - v2) * vecAct.y;
}

float timeCorrection(Object *part1, Object *part2)
{
    float px = part1->posi.x - part2->posi.x, py = part1->posi.y - part2->posi.y;
    float vx = part1->vlct.x - part2->vlct.x, vy = part1->vlct.y - part2->vlct.y;
    float r1r2 = part1->size + part2->size;
    float a = vx * vx + vy * vy, b = -2*(px * vx + py * vy), c = px * px + py * py - (r1r2 * r1r2);
    float delta = sqrtf(b * b - 4 * a * c);

    float t = ((-b - delta) / (2 * a));

    if(t < 0 || t > 1) 
        return 0.0;
    return t;
    
}

void applyCollision(World *worldstr)
{
    int totalCollisions = (worldstr->interactions[0] * 2) + 1;
    for(int i = 1; i < totalCollisions; i += 2)
    {
        int first = worldstr->interactions[i];
        int sec = worldstr->interactions[i+1];

        float t = timeCorrection(&worldstr->storage[first], &worldstr->storage[sec]);
        worldstr->storage[first].posi.x -= worldstr->storage[first].vlct.x * t;
        worldstr->storage[first].posi.y -= worldstr->storage[first].vlct.y * t;
        worldstr->storage[sec].posi.x -= worldstr->storage[sec].vlct.x * t;
        worldstr->storage[sec].posi.y -= worldstr->storage[sec].vlct.y * t;

        collisionFisics(&worldstr->storage[first], &worldstr->storage[sec], E);

        worldstr->storage[first].posi.x += worldstr->storage[first].vlct.x * t;
        worldstr->storage[first].posi.y += worldstr->storage[first].vlct.y * t;
        worldstr->storage[sec].posi.x += worldstr->storage[sec].vlct.x * t;
        worldstr->storage[sec].posi.y += worldstr->storage[sec].vlct.y * t;
    }
}

//=========================================================================================================

//Funcoes de deteccao

int getCollisions(World *worldstr)
{
    Vector2 distance;
    int quant = 0;
    int *tmp;
    for(int i = 0; i < worldstr->count; i++)
    {
        for(int j = i+1; j < worldstr->count; j++)
        {
            distance.x = worldstr->storage[i].posi.x - worldstr->storage[j].posi.x;
            distance.y = worldstr->storage[i].posi.y - worldstr->storage[j].posi.y;
            if(vecModule(distance) < worldstr->storage[i].size + worldstr->storage[j].size)
            {
                tmp = NULL;
                quant++;
                if(worldstr->interactions[0] < quant)
                {
                    tmp = realloc(worldstr->interactions, sizeof(int) * (quant * 2 + 1));
                    if(tmp != NULL)
                        worldstr->interactions = tmp;
                    else
                        return 0;
                }
                worldstr->interactions[quant*2-1] = i;
                worldstr->interactions[quant*2] = j;
            }
        }
    }
    worldstr->interactions[0] = quant;
    return 1;
}

void collisionOnWall(World *worldstr)
{
    for(int i = 0; i < worldstr->count; i++)
    {
        if((worldstr->storage[i].posi.x + worldstr->storage[i].size > SCREEN_W && worldstr->storage[i].vlct.x > 0) || (worldstr->storage[i].posi.x - worldstr->storage[i].size < 0 && worldstr->storage[i].vlct.x < 0))
        {
            worldstr->storage[i].vlct.x = -(worldstr->storage[i].vlct.x);
        }
        else if((worldstr->storage[i].posi.y + worldstr->storage[i].size > SCREEN_H && worldstr->storage[i].vlct.y > 0) || (worldstr->storage[i].posi.y - worldstr->storage[i].size < 0 && worldstr->storage[i].vlct.y < 0))
        {
            worldstr->storage[i].vlct.y = -(worldstr->storage[i].vlct.y);
        }
    }
}

//=========================================================================================================

//Funcoes de inicializacao

World startStorage()
{
    World world;
    world.interactions = (int*) malloc(sizeof(int));
    world.interactions[0] = 0;
    world.storage = NULL;
    world.count = 0;
    return world;
}

int arrayOfObjects(Object *parts, World *worldstr, int size)
{
    Object *tmp = realloc(worldstr->storage, ((size + worldstr->count) * sizeof(Object)));
    if(tmp != NULL)
    {
        worldstr->storage = tmp;
        for(int i = 0; i < size; i++)
        {
            worldstr->storage[worldstr->count] = parts[i];
            worldstr->count++;
        }
        free(parts);
        return(1);
    }
    return(0);
}

void createAnObject(Object part, World *worldstr)
{
    Object *tmp = realloc(worldstr->storage, (worldstr->count + 1) * sizeof(Object));
    if (tmp != NULL)
    {
        worldstr->storage = tmp;
        worldstr->count++;
        worldstr->storage[worldstr->count - 1] = part;
    }
}

Object* readText(int *size)
{
    FILE *file = fopen(OBJECTS, "r");
    if (file == NULL)
    {
        printf("erro ao abrir");
        return NULL;
    }
    int index;
    char text[100];
    fgets(text, sizeof(text), file);

    sscanf(text, "%d", &index);
    *size = index;
    Object *objects = malloc(sizeof(Object) * index);
    index--;
    while (fgets(text, sizeof(text), file) != NULL && index >= 0)
    {
        Object temp;
        if(sscanf(text, "radius: %f, mass: %f, posiX: %f, posiY: %f, vlctX: %f, vlctY: %f", 
        &temp.size, &temp.mass, &temp.posi.x, &temp.posi.y, &temp.vlct.x, &temp.vlct.y) == 6)
        {
            objects[index] = temp;
            index--;
        }
    }

    fclose(file);

    return objects;
}

//=========================================================================================================

//Funcao de desenho na tela

void drawCircle(SDL_Renderer *renderer, Object *part)
{
    float posix = part->posi.x, posiy = part->posi.y;
    float radius = part->size;
    
    int maxX = (int)(posix + radius);
    int maxY = (int)(posiy + radius);

    for(int i = (int)(posix - radius); i <= maxX; i++)
    {
        for(int j = (int)(posiy - radius); j <= maxY; j++)
        {
            float distanceX = i - posix;
            float distanceY = j - posiy;
            float total = (distanceX * distanceX + distanceY * distanceY);
            if(total <= radius * radius)
            {
                SDL_RenderPoint(renderer, (float)i, (float)j);
            }
        }
    }
}

//=========================================================================================================

int main() 
{
    // Inicializacao do mundo
    World world = startStorage();
    int size;
    Object *objects = readText(&size);
    arrayOfObjects(objects, &world, size);

    // Inicializacao do programa
    float deltaTime = 1000.0 / HZ;
    unsigned int startFrame;
    unsigned int wastedTime;

    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* window = NULL;
    SDL_Renderer* renderer = NULL;


    SDL_CreateWindowAndRenderer("Colisões", SCREEN_W, SCREEN_H, 0, &window, &renderer);
    int running = 1;

    while(running)
    {
        startFrame = SDL_GetTicks();
        SDL_Event event;
        while(SDL_PollEvent(&event))
        {
            if(event.type == SDL_EVENT_QUIT)
            {
                running = 0;
            }
        }
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);

        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

        for(int i = 0; i < world.count; i++)
        {
            drawCircle(renderer, &world.storage[i]);
            float vx = world.storage[i].vlct.x;
            float vy = world.storage[i].vlct.y;
            world.storage[i].posi.x += vx * deltaTime/1000;
            world.storage[i].posi.y += vy * deltaTime/1000;
        }

        //Faz a fisica do programa
        getCollisions(&world);
        collisionOnWall(&world);
        applyCollision(&world);

        wastedTime = SDL_GetTicks() - startFrame;
        if(wastedTime < deltaTime)
            SDL_Delay(deltaTime - wastedTime);
        SDL_RenderPresent(renderer);
    }

    free(world.storage);
    free(world.interactions);

    SDL_DestroyWindow(window);
    SDL_Quit();
}
