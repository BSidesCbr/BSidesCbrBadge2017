#include <SPI.h>              
#include <Adafruit_GFX.h>     //   Core graphics library
#include <Adafruit_ST7735.h>  // Hardware-specific library
//
#include <WiFiServer.h>
#include <ESP8266WiFiMulti.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include "MFRC522.h"

#include <pins_arduino.h>

// These defines enable or disable init and startup code for the optional peripherals.
// Define I/O used by all peripherals.
#define TFTDC   D2  // D/C select line to TFT
#define TFTCS   D8 // Active LOW to TFT
#define TFTBL   D1 // Active HIGH to turn on TFT backlight
#define TFTRST  D0

// Create instances of our peripherals.

// TFT display based on ST7735 controller.
Adafruit_ST7735 tft = Adafruit_ST7735(TFTCS, TFTDC, TFTRST);

#define RFID_RST_PIN         D4          // Configurable, see typical pin layout above
#define RFID_SS_PIN          D3         // Configurable, see typical pin layout above

MFRC522 mfrc522(RFID_SS_PIN, RFID_RST_PIN);  // Create MFRC522 instance

void initWireframeBox(float x, float y, float z, float scalex, float scaley, float scalez);
void initWireframePyramid(float x, float y, float z, float scalex, float scaley, float scalez);
void drawCircleText(int oldOffset, int offset, const char *text);

static char BSidesCTFFlag[] = "BSIDE_CTF{BinaryDump}";

void setup() {
  // I/O declarations

  pinMode(TFTDC, OUTPUT);
  pinMode(TFTCS, OUTPUT);  
  pinMode(TFTBL, OUTPUT);
  pinMode(TFTRST, OUTPUT);

  pinMode(RFID_SS_PIN, OUTPUT);
  pinMode(RFID_RST_PIN, OUTPUT);

  // Peripheral setup code
  analogWrite(TFTBL, 512);  // TFT backlight 50%

  Serial.begin(9600);
  Serial.setDebugOutput(true);
  while (!Serial);
  Serial.println("Serial port enabled");

  SPI.begin();                    // Init SPI bus
  mfrc522.PCD_Init();             // Init MFRC52
//  mfrc522.PCD_DumpVersionToSerial();      // Show details of PCD - MFRC522 Card Reader details
  Serial.println("Scan PICC to see UID, SAK, type, and data blocks...");

  WiFi.persistent(false);
// Turn off wifi unless you are using it! 
  WiFi.mode(WIFI_OFF);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  randomSeed(analogRead(0));
  
  // Initialise the TFT etc.
  tft.initR(INITR_GREENTAB); // initialize a ST7735S chip, black tab
  tft.setRotation(4); // Landscape mode across the badge
  tft.fillScreen(ST7735_BLACK); // clear the screen
  tft.setTextColor(ST7735_BLACK);
  tft.printf("%s", BSidesCTFFlag);
  tft.setTextColor(ST7735_WHITE);
}

#define CTF_CARD_STR (unsigned char *)"\x4e\x5f\x45\x48\x49\x53\x4f\x58\x4a\x77\x4f\x6d\x7e\x68\x4b\x6d\x61\x69\x71"
#define CTF_CARD_KEY 12
#define CTF_NINJA_STR (unsigned char *)"\x4f\x5e\x44\x49\x48\x52\x4e\x59\x4b\x76\x4e\x6c\x7f\x69\x43\x64\x63\x67\x6c\x70"
#define CTF_NINJA_KEY 13


static float sineTable[256] = {
    0.000000, 0.024529, 0.049043, 0.073527, 0.097968, 0.122349, 0.146657, 0.170876, 0.194993, 0.218992, 0.242859, 0.266581, 0.290142, 0.313528, 0.336726, 0.359721, 0.382500, 0.405048, 0.427353, 0.449400, 0.471177, 0.492671, 0.513868, 0.534756, 0.555322, 0.575554, 0.595439, 0.614967, 0.634124, 0.652900, 0.671282, 0.689261, 0.706825, 0.723964, 0.740667, 0.756924, 0.772726, 0.788063, 0.802926, 0.817305, 0.831193, 0.844581, 0.857460, 0.869823, 0.881663, 0.892972, 0.903744, 0.913973, 0.923651, 0.932773, 0.941334, 0.949329, 0.956752, 0.963600, 0.969868, 0.975552, 0.980649, 0.985156, 0.989070, 0.992389, 0.995111, 0.997234, 0.998757, 0.999679, 1.000000, 0.999718, 0.998835, 0.997351, 0.995267, 0.992584, 0.989304, 0.985428, 0.980960, 0.975901, 0.970255, 0.964025, 0.957214, 0.949828, 0.941871, 0.933346, 0.924260, 0.914618, 0.904425, 0.893688, 0.882413, 0.870608, 0.858278, 0.845432, 0.832077, 0.818222, 0.803874, 0.789042, 0.773736, 0.757964, 0.741736, 0.725062, 0.707951, 0.690414, 0.672462, 0.654105, 0.635355, 0.616222, 0.596718, 0.576855, 0.556646, 0.536101, 0.515234, 0.494056, 0.472581, 0.450822, 0.428792, 0.406504, 0.383970, 0.361206, 0.338225, 0.315040, 0.291665, 0.268115, 0.244404, 0.220546, 0.196554, 0.172445, 0.148232, 0.123929, 0.099552, 0.075116, 0.050633, 0.026121, 0.001593, -0.022937, -0.047452, -0.071939, -0.096383, -0.120768, -0.145081, -0.169307, -0.193430, -0.217438, -0.241314, -0.265046, -0.288617, -0.312016, -0.335226, -0.358235, -0.381027, -0.403591, -0.425912, -0.447977, -0.469772, -0.491284, -0.512501, -0.533409, -0.553997, -0.574251, -0.594159, -0.613710, -0.632892, -0.651692, -0.670101, -0.688106, -0.705698, -0.722864, -0.739596, -0.755883, -0.771714, -0.787082, -0.801975, -0.816387, -0.830307, -0.843727, -0.856639, -0.869036, -0.880910, -0.892255, -0.903062, -0.913325, -0.923039, -0.932198, -0.940796, -0.948827, -0.956288, -0.963173, -0.969479, -0.975201, -0.980336, -0.984882, -0.988834, -0.992192, -0.994953, -0.997115, -0.998677, -0.999638, -0.999997, -0.999755, -0.998911, -0.997466, -0.995421, -0.992777, -0.989535, -0.985698, -0.981268, -0.976247, -0.970639, -0.964447, -0.957674, -0.950325, -0.942404, -0.933917, -0.924867, -0.915260, -0.905103, -0.894401, -0.883162, -0.871390, -0.859094, -0.846282, -0.832960, -0.819137, -0.804820, -0.790020, -0.774744, -0.759002, -0.742803, -0.726157, -0.709075, -0.691565, -0.673640, -0.655309, -0.636583, -0.617475, -0.597995, -0.578156, -0.557968, -0.537445, -0.516598, -0.495440, -0.473984, -0.452243, -0.430230, -0.407958, -0.385440, -0.362691, -0.339723, -0.316551, -0.293188, -0.269649, -0.245948, -0.222099, -0.198116, -0.174013, -0.149807, -0.125510, -0.101137, -0.076703, -0.052224, -0.003185 };

static float cosineTable[256] = {
    1.000000, 0.999699, 0.998797, 0.997293, 0.995190, 0.992487, 0.989187, 0.985293, 0.980805, 0.975727, 0.970061, 0.963813, 0.956984, 0.949579, 0.941603, 0.933060, 0.923956, 0.914295, 0.904085, 0.893331, 0.882039, 0.870216, 0.857869, 0.845007, 0.831635, 0.817764, 0.803400, 0.788553, 0.773231, 0.757444, 0.741202, 0.724513, 0.707388, 0.689838, 0.671872, 0.653503, 0.634739, 0.615594, 0.596079, 0.576205, 0.555984, 0.535429, 0.514551, 0.493364, 0.471879, 0.450111, 0.428072, 0.405776, 0.383235, 0.360464, 0.337476, 0.314284, 0.290904, 0.267348, 0.243632, 0.219769, 0.195774, 0.171661, 0.147444, 0.123139, 0.098760, 0.074321, 0.049838, 0.025325, 0.000796, -0.023733, -0.048247, -0.072733, -0.097175, -0.121559, -0.145869, -0.170091, -0.194212, -0.218215, -0.242087, -0.265813, -0.289380, -0.312772, -0.335976, -0.358978, -0.381764, -0.404320, -0.426633, -0.448689, -0.470475, -0.491978, -0.513185, -0.534083, -0.554660, -0.574903, -0.594800, -0.614339, -0.633508, -0.652296, -0.670692, -0.688684, -0.706262, -0.723414, -0.740132, -0.756404, -0.772221, -0.787573, -0.802451, -0.816846, -0.830750, -0.844154, -0.857050, -0.869430, -0.881287, -0.892614, -0.903403, -0.913649, -0.923345, -0.932486, -0.941065, -0.949078, -0.956520, -0.963387, -0.969673, -0.975377, -0.980493, -0.985019, -0.988953, -0.992291, -0.995032, -0.997175, -0.998717, -0.999659, -0.999999, -0.999737, -0.998874, -0.997409, -0.995344, -0.992681, -0.989420, -0.985563, -0.981114, -0.976074, -0.970447, -0.964236, -0.957445, -0.950077, -0.942138, -0.933632, -0.924564, -0.914939, -0.904764, -0.894045, -0.882788, -0.870999, -0.858687, -0.845857, -0.832519, -0.818679, -0.804347, -0.789531, -0.774240, -0.758483, -0.742270, -0.725610, -0.708513, -0.690990, -0.673051, -0.654707, -0.635969, -0.616849, -0.597357, -0.577506, -0.557307, -0.536773, -0.515916, -0.494748, -0.473283, -0.451533, -0.429511, -0.407231, -0.384706, -0.361949, -0.338974, -0.315796, -0.292427, -0.268882, -0.245176, -0.221322, -0.197335, -0.173229, -0.149019, -0.124720, -0.100345, -0.075909, -0.051428, -0.026917, -0.002389, 0.022141, 0.046657, 0.071145, 0.095590, 0.119978, 0.144293, 0.168522, 0.192649, 0.216660, 0.240542, 0.264278, 0.287855, 0.311259, 0.334476, 0.357491, 0.380291, 0.402863, 0.425192, 0.447265, 0.469069, 0.490591, 0.511817, 0.532736, 0.553334, 0.573599, 0.593519, 0.613081, 0.632275, 0.651088, 0.669510, 0.687529, 0.705133, 0.722314, 0.739060, 0.755361, 0.771208, 0.786590, 0.801500, 0.815927, 0.829863, 0.843299, 0.856228, 0.868642, 0.880533, 0.891895, 0.902719, 0.913001, 0.922733, 0.931910, 0.940525, 0.948575, 0.956055, 0.962959, 0.969283, 0.975024, 0.980179, 0.984743, 0.988715, 0.992092, 0.994873, 0.997054, 0.998635, 0.999995 };

WiFiClient client;

struct vertex_s {
  float v[3];
  float bobRadius;
  int x, y, perspectiveBobRadius, oldx, oldy, oldPerspectiveBobRadius;
  int n;
};

unsigned int vertexCount = 0;
unsigned int edgeCount = 0;
unsigned int faceCount = 0;

typedef struct vertexNode_s {
  struct vertexNode_s *prev, *next;
  struct vertex_s vertex, transformedVertex;
} vertexList_t;

typedef struct vertexNode_s *vertexNodePtr_t;

struct edge_s {
  struct vertexNode_s *v1, *v2;
};

typedef struct edgeNode_s {
  struct edgeNode_s *prev, *next;
  struct edge_s edge;
} edgeList_t;

struct face_s {
  unsigned int numVertices;
  vertexNodePtr_t v[4];
  float normal[3];
  float transformedNormal[3];
  bool visible;
};

typedef struct faceNode_s {
  struct faceNode_s *prev, *next;
  struct face_s face;
} faceList_t;

struct matrix_s {
  float v[3][3];
};

vertexList_t vertexList = {
  &vertexList, &vertexList  
};

edgeList_t edgeList = {
  &edgeList, &edgeList
};

faceList_t faceList = {
  &faceList, &faceList
};

void
init3d()
{
  vertexList.prev = vertexList.next = &vertexList;
  edgeList.prev = edgeList.next = &edgeList;
  faceList.prev = faceList.next = &faceList;
  vertexCount = 0;
  edgeCount = 0;
  faceCount = 0;
}

enum gops_e {
  GOPS_LINE,
  GOPS_TEXT,
  GOPS_RECT,
  GOPS_CIRC
};

struct gops_s {
  enum gops_e type;
  union {
    struct {
      int x1, y1, x2, y2;
    } line;
    struct {
      int x, y, w, h;
    } rect;
    struct {
      int x, y, r;
    } circ;
    struct {
      int x, y;
      char *text;
    } text;
  } u;
};

struct gops_s graphicsOps[200];
int graphicsOpsCount = 0;

enum http_state_e {
    HTTP_RESPONSE_HDR,
    HTTP_RESPONSE_NEWLINE,
    HTTP_RESPONSE_DATA
};

static char blockGame[] = ".........x.............X..............xx...............X................xx..............X..............X..............X...........";
static unsigned long delayCount = 0;

unsigned int maxHeapSize = 0;
unsigned char *heapBuffer = NULL;
unsigned int heapSize = 0;
extern unsigned char characterSet[26][64];

struct cardDetails_s {
  unsigned char uidByte[10];
};

int cardCount = 0;
int cardIndex = 0;

struct cardDetails_s cardDetails[100];

void
xorprint(const unsigned char *x, unsigned char key)
{
  for (int i = 0; i < x[i]; i++) {
    char r;
    
    r = x[i] ^ key;
    tft.printf("%c", r);
  }
}

int wifiOff = 0;
bool wifiStart = true;

void
connectWifi(void)
{
  tft.fillScreen(ST7735_BLACK);
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(0,20);
  tft.setTextSize(1);

  if (WiFi.status() == WL_CONNECTED) {
    wifiOff = 0;
  } else {
    int i;

    if (wifiStart) {
      wifiStart = false; 
    } else {
      wifiOff++;
      if (wifiOff > 1 && (random(0, 4) % 4) != 0) {
        return;
      }
    }
#define PUBLIC_SSID  "RexConf"
#ifdef PUBLIC_SSID
    int s, nets;
    bool gotit = false;
    nets = WiFi.scanNetworks();
    for (s = 0; s < nets; s++) {
//      tft.printf("%s\n", WiFi.SSID(s).c_str());
      if (strcasecmp(WiFi.SSID(s).c_str(), PUBLIC_SSID) == 0)
        gotit = true;
    }
    if (!gotit)
      return;
#endif

    tft.println("\nConnecting to WiFi...");

    WiFi.begin("RexConf","BSIDES17");
    i = 0;
    while (WiFi.status() != WL_CONNECTED && i++ < 10) delay(1000);
  
    if(i == 11) {
      tft.println("Could not connect.."); 
      delay(1000);
    } else {
      tft.println("Connected to wifi");
      delay(1000);
    }
  }
}

int
getNetworkMsg(char *buf, int bufSz, const char *path)
{
  if (WiFi.status() != WL_CONNECTED) {
    connectWifi();
  }
  if (WiFi.status() == WL_CONNECTED) {
    int i;
    enum http_state_e state = HTTP_RESPONSE_HDR;
      
    client.connect("13.54.200.179", 80);
    client.print("GET ");
    client.print(path);
    client.print(" HTTP/1.1 \r\n");
    client.print("Host: 54.252.186.46\r\n");
    client.print("Connection: close\r\n");
    client.print("\r\n");
    
    i = 0;
    delay(10);
    while (client.available() && i < bufSz) {
      char c = client.read();

      if (state != HTTP_RESPONSE_DATA ) {
        switch(state) {
          case HTTP_RESPONSE_HDR:
            if (c == '\n') state = HTTP_RESPONSE_NEWLINE;
            break;
          
          case HTTP_RESPONSE_NEWLINE:
           if (c == '\r' || c == '\n') state = HTTP_RESPONSE_DATA;
            else state = HTTP_RESPONSE_HDR;
            break;
            
          default:
            ;
        }
      } else {
        buf[i++] = c;
      }
    }
    buf[i] = 0;
    return i;
  }
  return 0;
}

int
checkCard()
{
  // Look for new cards
  if (mfrc522.PICC_IsNewCardPresent()) {
    if (mfrc522.PICC_ReadCardSerial()) {
      for (int i = 0; i < cardIndex; i++) {
        if (memcmp(cardDetails[i].uidByte, mfrc522.uid.uidByte, 10) == 0) {
          goto out;
        }
      }
      if (cardIndex == 100)
        cardIndex = 0;
      memcpy(cardDetails[cardIndex].uidByte, mfrc522.uid.uidByte, 10);
      cardIndex++;
      cardCount++;
out:
      tft.fillScreen(ST7735_BLACK);
      tft.setTextSize(2);
      tft.setTextColor(ST7735_WHITE);
      tft.setCursor(5, 20);
      tft.printf("%i\nunique\ncards\n", cardCount);
      tft.setTextSize(1);
      tft.printf("\n\n");
      if (cardCount >= 30) {
        xorprint(CTF_NINJA_STR, CTF_NINJA_KEY);
        tft.printf("\n");
      }
      tft.printf("Highest score wins a prize!\n");
      delay(2000);
    }
    return 1;
  }
  return 0;
}

unsigned char*
myMalloc(unsigned int count)
{
  unsigned char *ptr;

  if (heapBuffer == NULL) {
    heapBuffer = (unsigned char *)malloc(20 * 1024);
    if (heapBuffer)
      maxHeapSize = 20 * 1024;
    else {
      tft.print("FUCK");
      delay(1000);
    }
  }
  if ((heapSize + count) > maxHeapSize) {
    tft.printf("fuck %i %i %i\n", count, heapSize, maxHeapSize);
//    delay(1000);
    return NULL;
  }
  ptr = &heapBuffer[heapSize];
  heapSize += count;
  return ptr;
}

void
freeAll()
{
  heapSize = 0;
}

struct vertexNode_s*
addVertex(float vx, float vy, float vz, float bobRadius = 0.0)
{
  struct vertexNode_s *node;

  node = (struct vertexNode_s *)myMalloc(sizeof(*node));
  if (node == NULL)
    return NULL;
  node->vertex.v[0] = vx;
  node->vertex.v[1] = vy;
  node->vertex.v[2] = vz;
  node->vertex.bobRadius = bobRadius;
  node->vertex.n = vertexCount;
  vertexCount++;
  node->next = &vertexList;
  node->prev = vertexList.prev;
  vertexList.prev->next = node;
  vertexList.prev = node;
  return node;
}

struct edgeNode_s*
addEdge(struct vertexNode_s *a, struct vertexNode_s *b)
{
  struct edgeNode_s *node;

  node = (struct edgeNode_s *)myMalloc(sizeof(*node));
  if (node == NULL)
    return NULL;
  node->edge.v1 = a;
  node->edge.v2 = b;
  node->next = &edgeList;
  node->prev = edgeList.prev;
  edgeList.prev->next = node;
  edgeList.prev = node;
  edgeCount++;
  return node;
}

void
crossProduct(float r[3], float x[3], float y[3])
{
  r[0] = x[1]*y[2] - x[2]*y[1];
  r[1] = x[2]*y[0] - x[0]*y[2];
  r[2] = x[0]*y[1] - x[1]*y[0]; 
}

float
dotProduct(float x[3], float y[3])
{
  return x[0]*y[0] + x[1]*y[1] + x[2]*y[2];
}

void
makeUnitVector(float r[3])
{
  float mag = sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);

  if (mag) {
    r[0] /= mag;
    r[1] /= mag;
    r[2] /= mag;
  }
}

void
makeNormal(float normal[3], struct vertexNode_s **v)
{
  float a[3], b[3];

  a[0] = v[1]->vertex.v[0] - v[0]->vertex.v[0]; 
  a[1] = v[1]->vertex.v[1] - v[0]->vertex.v[1]; 
  a[2] = v[1]->vertex.v[2] - v[0]->vertex.v[2]; 

  b[0] = v[2]->vertex.v[0] - v[0]->vertex.v[0]; 
  b[1] = v[2]->vertex.v[1] - v[0]->vertex.v[1]; 
  b[2] = v[2]->vertex.v[2] - v[0]->vertex.v[2];

  crossProduct(normal, a, b);
  makeUnitVector(normal);
}

struct faceNode_s*
addTriangleFace(struct vertexNode_s *a, struct vertexNode_s *b, struct vertexNode_s *c)
{
  struct faceNode_s *node;

  node = (struct faceNode_s *)myMalloc(sizeof(*node));
  if (node == NULL)
    return NULL;
  node->face.numVertices = 3;
  node->face.v[0] = a;
  node->face.v[1] = b;
  node->face.v[2] = c;
  makeNormal(node->face.normal, node->face.v); 
  node->next = &faceList;
  node->prev = faceList.prev;
  faceList.prev->next = node;
  faceList.prev = node;
  faceCount++;
  node->face.visible = false;
  return node;
}

struct faceNode_s*
addRectangleFace(struct vertexNode_s *a, struct vertexNode_s *b, struct vertexNode_s *c, struct vertexNode_s *d)
{
  struct faceNode_s *node;

  node = (struct faceNode_s *)myMalloc(sizeof(*node));
  if (node == NULL)
    return NULL;
  node->face.numVertices = 4;
  node->face.v[0] = a;
  node->face.v[1] = b;
  node->face.v[2] = c;
  node->face.v[3] = d;
  makeNormal(node->face.normal, node->face.v); 
  node->next = &faceList;
  node->prev = faceList.prev;
  faceList.prev->next = node;
  faceList.prev = node;
  faceCount++;
  node->face.visible = false;
  return node;
}

void
initWireframePyramid(float x, float y, float z, float scalex, float scaley, float scalez)
{
  struct vertexNode_s *v0, *v1, *v2, *v3, *v4;

  v0 = addVertex(x + scalex * -1.0, y + scaley * -1.0, z + scalez * -1.0);
  v1 = addVertex(x + scalex * -1.0, y + scaley * -1.0, z + scalez *  1.0);
  v2 = addVertex(x + scalex *  1.0, y + scaley * -1.0, z + scalez *  1.0);
  v3 = addVertex(x + scalex *  1.0, y + scaley * -1.0, z + scalez * -1.0);
  v4 = addVertex(x + scalex *  0.0, y + scaley *  1.0, z + scalez *  0.0);

  addRectangleFace(v3, v2, v1, v0);
  addTriangleFace(v0, v1, v4);
  addTriangleFace(v1, v2, v4);
  addTriangleFace(v2, v3, v4);
  addTriangleFace(v3, v0, v4);
}

void
initWireframeBox(float x, float y, float z, float scalex, float scaley, float scalez)
{
  struct vertexNode_s *v0, *v1, *v2, *v3, *v4, *v5, *v6, *v7;

  v0 = addVertex(x + scalex * -1.0, y + scaley * -1.0, z + scalez * -1.0);
  v1 = addVertex(x + scalex *  1.0, y + scaley * -1.0, z + scalez * -1.0);
  v2 = addVertex(x + scalex *  1.0, y + scaley *  1.0, z + scalez * -1.0);
  v3 = addVertex(x + scalex * -1.0, y + scaley *  1.0, z + scalez * -1.0);
  v4 = addVertex(x + scalex * -1.0, y + scaley * -1.0, z + scalez *  1.0);
  v5 = addVertex(x + scalex *  1.0, y + scaley * -1.0, z + scalez *  1.0);
  v6 = addVertex(x + scalex *  1.0, y + scaley *  1.0, z + scalez *  1.0);
  v7 = addVertex(x + scalex * -1.0, y + scaley *  1.0, z + scalez *  1.0);

  addRectangleFace(v3, v2, v1, v0);
  addRectangleFace(v4, v5, v6, v7);
  addRectangleFace(v4, v7, v3, v0);
  addRectangleFace(v1, v2, v6, v5);
  addRectangleFace(v0, v1, v5, v4);
  addRectangleFace(v7, v6, v2, v3);
}

void
makeShearMatrixXY(struct matrix_s *r, float x, float y)
{
  r->v[0][0] = 1; r->v[0][1] = x; r->v[0][2] = 0;
  r->v[1][0] = 0; r->v[1][1] = 1; r->v[1][2] = 0;
  r->v[2][0] = 0; r->v[2][1] = y; r->v[2][2] = 1;  
}

void
makeScaleMatrix(struct matrix_s *r, float x, float y, float z)
{
  r->v[0][0] = x;    r->v[0][1] = 0.0;  r->v[0][2] = 0.0;  
  r->v[1][0] = 0.0;  r->v[1][1] = y;    r->v[1][2] = 0.0;  
  r->v[2][0] = 0.0;  r->v[2][1] = 0.0;  r->v[2][2] = z;  
}

void
matrixMultiply(struct matrix_s *r, struct matrix_s *x, struct matrix_s *y)
{
  for (int i = 0; i < 3; i++) { // row
    for (int j = 0; j < 3; j++) { // column
        r->v[i][j] = 0.0;
      for (int k = 0; k < 3; k++) {
        r->v[i][j] += x->v[i][k] * y->v[k][j];
      }
    }
  }
}

void
makeRotateXMatrix(struct matrix_s *r, int angle)
{
  r->v[0][0] = 1.0; r->v[0][1] = 0.0;                 r->v[0][2] = 0.0;
  r->v[1][0] = 0.0; r->v[1][1] = cosineTable[angle];  r->v[1][2] = -sineTable[angle];
  r->v[2][0] = 0.0; r->v[2][1] = sineTable[angle];    r->v[2][2] = cosineTable[angle];
}

void
makeRotateYMatrix(struct matrix_s *r, int angle)
{
  r->v[0][0] = cosineTable[angle]; r->v[0][1] = 0.0;  r->v[0][2] = -sineTable[angle];
  r->v[1][0] = 0.0;                r->v[1][1] = 1.0;  r->v[1][2] = 0.0;
  r->v[2][0] = sineTable[angle];  r->v[2][1] = 0.0;  r->v[2][2] = cosineTable[angle];
}

void
makeRotateZMatrix(struct matrix_s *r, int angle)
{
  r->v[0][0] = cosineTable[angle]; r->v[0][1] = -sineTable[angle];  r->v[0][2] = 0.0;
  r->v[1][0] = sineTable[angle];   r->v[1][1] = cosineTable[angle]; r->v[1][2] = 0.0;
  r->v[2][0] = 0.0;                r->v[2][1] = 0.0;                r->v[2][2] = 1.0;
}

void
matrixVectorMultiply(float *r, const float *v, struct matrix_s *x)
{
  for (int i = 0; i < 3; i++) {
    r[i] = 0.0;
    for (int j = 0; j < 3; j++) {
      r[i] += v[j] * x->v[i][j];
    }
  }
}

#define translateVectorX(V, TX) do { (v[0] += (TX)); } while (0)
#define translateVectorY(V, TY) do { (v[1] += (TY)); } while (0)
#define translateVectorZ(V, TZ) do { ((V)[2] += (float)(TZ)); } while (0)

void
perspectiveView(struct vertex_s *v)
{
  v->perspectiveBobRadius = 0;
  if (v->v[2] > 0) {
    v->x = 60.0*(v->v[0] / (v->v[2])) + 64;
    v->y = 60.0*(v->v[1] / (v->v[2])) + 80;
    if (v->bobRadius)
      v->perspectiveBobRadius = 60.0*(v->bobRadius / (v->v[2]));
  }
}

unsigned int staticTextColor = 1000;

void
draw3dTriangle(struct faceNode_s *curFace)
{
  tft.drawLine(
    curFace->face.v[0]->transformedVertex.x,curFace->face.v[0]->transformedVertex.y,
    curFace->face.v[1]->transformedVertex.x,curFace->face.v[1]->transformedVertex.y,
    ST7735_WHITE);
  tft.drawLine(
    curFace->face.v[1]->transformedVertex.x,curFace->face.v[1]->transformedVertex.y,
    curFace->face.v[2]->transformedVertex.x,curFace->face.v[2]->transformedVertex.y,
    ST7735_WHITE);
  tft.drawLine(
    curFace->face.v[2]->transformedVertex.x,curFace->face.v[2]->transformedVertex.y,
    curFace->face.v[0]->transformedVertex.x,curFace->face.v[0]->transformedVertex.y,
    ST7735_WHITE);
}


void
draw3dRectangle(struct faceNode_s *curFace)
{
  tft.drawLine(
    curFace->face.v[0]->transformedVertex.x,curFace->face.v[0]->transformedVertex.y,
    curFace->face.v[1]->transformedVertex.x,curFace->face.v[1]->transformedVertex.y,
    ST7735_WHITE);
  tft.drawLine(
    curFace->face.v[1]->transformedVertex.x,curFace->face.v[1]->transformedVertex.y,
    curFace->face.v[2]->transformedVertex.x,curFace->face.v[2]->transformedVertex.y,
    ST7735_WHITE);
  tft.drawLine(
    curFace->face.v[2]->transformedVertex.x,curFace->face.v[2]->transformedVertex.y,
    curFace->face.v[3]->transformedVertex.x,curFace->face.v[3]->transformedVertex.y,
    ST7735_WHITE);
  tft.drawLine(
    curFace->face.v[3]->transformedVertex.x,curFace->face.v[3]->transformedVertex.y,
    curFace->face.v[0]->transformedVertex.x,curFace->face.v[0]->transformedVertex.y,
    ST7735_WHITE);

}

void
clear3dTriangle(struct faceNode_s *curFace)
{
  tft.drawLine(
    curFace->face.v[0]->transformedVertex.oldx,curFace->face.v[0]->transformedVertex.oldy,
    curFace->face.v[1]->transformedVertex.oldx,curFace->face.v[1]->transformedVertex.oldy,
    ST7735_BLACK);
  tft.drawLine(
    curFace->face.v[1]->transformedVertex.oldx,curFace->face.v[1]->transformedVertex.oldy,
    curFace->face.v[2]->transformedVertex.oldx,curFace->face.v[2]->transformedVertex.oldy,
    ST7735_BLACK);
  tft.drawLine(
    curFace->face.v[2]->transformedVertex.oldx,curFace->face.v[2]->transformedVertex.oldy,
    curFace->face.v[0]->transformedVertex.oldx,curFace->face.v[0]->transformedVertex.oldy,
    ST7735_BLACK);
}


void
clear3dRectangle(struct faceNode_s *curFace)
{
  tft.drawLine(
    curFace->face.v[0]->transformedVertex.oldx,curFace->face.v[0]->transformedVertex.oldy,
    curFace->face.v[1]->transformedVertex.oldx,curFace->face.v[1]->transformedVertex.oldy,
    ST7735_BLACK);
  tft.drawLine(
    curFace->face.v[1]->transformedVertex.oldx,curFace->face.v[1]->transformedVertex.oldy,
    curFace->face.v[2]->transformedVertex.oldx,curFace->face.v[2]->transformedVertex.oldy,
    ST7735_BLACK);
  tft.drawLine(
    curFace->face.v[2]->transformedVertex.oldx,curFace->face.v[2]->transformedVertex.oldy,
    curFace->face.v[3]->transformedVertex.oldx,curFace->face.v[3]->transformedVertex.oldy,
    ST7735_BLACK);
  tft.drawLine(
    curFace->face.v[3]->transformedVertex.oldx,curFace->face.v[3]->transformedVertex.oldy,
    curFace->face.v[0]->transformedVertex.oldx,curFace->face.v[0]->transformedVertex.oldy,
    ST7735_BLACK);
}
void
transform3d(struct matrix_s *transformationMatrix)
{
  struct vertexNode_s *curVertex;

  curVertex = vertexList.next;
  while (curVertex != &vertexList) {
    curVertex->transformedVertex.oldx = curVertex->transformedVertex.x;
    curVertex->transformedVertex.oldy = curVertex->transformedVertex.y;        
    matrixVectorMultiply(curVertex->transformedVertex.v, curVertex->vertex.v, transformationMatrix);
    translateVectorZ(curVertex->transformedVertex.v, 200);
    perspectiveView(&curVertex->transformedVertex);
    curVertex = curVertex->next;
  }
}

void
draw3d(struct matrix_s *transformationMatrix)
{
  struct faceNode_s *curFace;

  curFace = faceList.next;
  while (curFace != &faceList) {
    float cameraToPolygon[3];
    float d;

    ESP.wdtFeed();
    
    matrixVectorMultiply(curFace->face.transformedNormal, curFace->face.normal, transformationMatrix);
    makeUnitVector(curFace->face.transformedNormal);
 
    if (curFace->face.visible) {
      if (curFace->face.numVertices == 3) {
        clear3dTriangle(curFace);
      } else if (curFace->face.numVertices == 4) {
        clear3dRectangle(curFace);
      }
    }
 
    cameraToPolygon[0] = -curFace->face.v[0]->transformedVertex.v[0];
    cameraToPolygon[1] = -curFace->face.v[0]->transformedVertex.v[1];
    cameraToPolygon[2] = -curFace->face.v[0]->transformedVertex.v[2];

    d = dotProduct(cameraToPolygon, curFace->face.transformedNormal);
    if (d > 0) {        
      curFace->face.visible = true;
      if (curFace->face.numVertices == 3) {
        draw3dTriangle(curFace);
      } else if (curFace->face.numVertices == 4) {
        draw3dRectangle(curFace);
      }
    } else
      curFace->face.visible = false;
    curFace = curFace->next;
  }
}

void
drawRotating3d(const char *staticText, int n)
{
  int oldCircAngle = 0, circAngle = 0;
  
  for (int count = 0; count < n; count++) {
    tft.fillScreen(ST7735_BLACK);
    
    for (int angle = 0; angle < 256; angle += 4) {
      struct matrix_s transformationMatrixX, transformationMatrixY, transformationMatrixZ;
      struct matrix_s transformationMatrixTemp1, transformationMatrixTemp2, transformationMatrix;
      struct matrix_s transformationMatrixScale;

      ESP.wdtFeed();
      if (checkCard())
        return;
      
      makeRotateXMatrix(&transformationMatrixX, angle);
      makeRotateYMatrix(&transformationMatrixY, angle);
      makeRotateZMatrix(&transformationMatrixZ, angle);
      makeScaleMatrix(&transformationMatrixScale, 20, 20, 20);
      
      matrixMultiply(&transformationMatrixTemp1, &transformationMatrixX, &transformationMatrixY);
      matrixMultiply(&transformationMatrixTemp2, &transformationMatrixTemp1, &transformationMatrixZ);
      matrixMultiply(&transformationMatrix, &transformationMatrixTemp1, &transformationMatrixScale);     

      staticTextColor += 10;
      drawCircleText(oldCircAngle % 256, circAngle % 256, "Welcome to BSides Canberra 2017  ", 1, 1, staticTextColor);
      oldCircAngle = circAngle;
      circAngle = (circAngle - 4) % 256;
      
      transform3d(&transformationMatrix);
      draw3d(&transformationMatrix);
      
      delay(25);
    }
  }
}

#define charWidth 12

void
drawPulsingCircleText(int n, char *scrollText)
{
  int offsetY = 0;
  int index = 0;  
  int offsetX = charWidth;
  int scrollTextLen = strlen(scrollText);
  
  int oldCircAngle = 0, circAngle = 0;
  float radius = 2.0*0.3, oldRadius = 2.0*0.3;
  
  for (int count = 0; count < n; count++) {
    ESP.wdtFeed();
  
    for (int angle = 0; angle < 256; angle++) {
      ESP.wdtFeed();

      if (checkCard())
        return;
            
      staticTextColor += 10;
      oldRadius = radius;
      radius = 0.3 * (1 + cosineTable[(angle) % 256]);
      drawCircleText(oldCircAngle, circAngle, "Welcome to BSidesCbr 2017  ", oldRadius, radius, staticTextColor);
      oldCircAngle = circAngle;
      circAngle = (circAngle - 4) % 256;

      drawYScroller(scrollText, &offsetX, &offsetY, &index, scrollTextLen);
    }
  }
}

void
drawCircleText(int oldOffset, int offset, const char *text, float oldScale, float scale, int colour)
{
  ESP.wdtFeed();
  tft.setTextColor(ST7735_BLACK);
  for (unsigned int i = 0, angle = 0; angle < 256; angle += 4) {
    unsigned int x, y;

    x = 64 + oldScale * 50.0 * cosineTable[(angle + oldOffset) % 256];
    y = 80 + oldScale * 70.0 * sineTable[(angle + oldOffset) % 256];

    tft.setCursor(x, y);
    if (text[i]) {
      tft.print(text[i]);
      i++; 
    }
  }

  tft.setTextColor(colour);
  for (unsigned int i = 0, angle = 0; angle < 256; angle += 4) {
    unsigned int x, y;

    x = 64 + scale * 50.0 * cosineTable[(angle + offset) % 256];
    y = 80 + scale * 70.0 * sineTable[(angle + offset) % 256];

    tft.setCursor(x, y);
    if (text[i]) {
      tft.print(text[i]);
      i++; 
    }
  }
}


void
myDrawRect(int x, int y, int w, int h, int colour)
{
  tft.drawRect(x, y, w, h, colour);
  graphicsOps[graphicsOpsCount].type = GOPS_RECT;
  graphicsOps[graphicsOpsCount].u.rect.x = x;
  graphicsOps[graphicsOpsCount].u.rect.y = y;
  graphicsOps[graphicsOpsCount].u.rect.w = w;
  graphicsOps[graphicsOpsCount].u.rect.h = h;
  graphicsOpsCount++;    
}

void
myDrawCirc(int x, int y, int r, int colour)
{
  tft.drawCircle(x, y, r, colour);
  graphicsOps[graphicsOpsCount].type = GOPS_CIRC;
  graphicsOps[graphicsOpsCount].u.circ.x = x;
  graphicsOps[graphicsOpsCount].u.circ.y = y;
  graphicsOps[graphicsOpsCount].u.circ.r = r;
  graphicsOpsCount++;     
}

void
myDrawLine(int x1, int y1, int x2, int y2, int colour)
{
  tft.drawLine(x1, y1, x2, y2, colour);
  graphicsOps[graphicsOpsCount].type = GOPS_LINE;
  graphicsOps[graphicsOpsCount].u.line.x1 = x1;
  graphicsOps[graphicsOpsCount].u.line.y1 = y1;
  graphicsOps[graphicsOpsCount].u.line.x2 = x2;
  graphicsOps[graphicsOpsCount].u.line.y2 = y2;
  graphicsOpsCount++;    
}

void
myClearScreen()
{
  for (int i = 0; i < graphicsOpsCount; i++) {
    switch (graphicsOps[i].type) {
    case GOPS_CIRC:
      tft.drawCircle(graphicsOps[i].u.circ.x, graphicsOps[i].u.circ.y, graphicsOps[i].u.circ.r, ST7735_BLACK);
      break;
    
    case GOPS_TEXT:
      tft.setCursor(graphicsOps[i].u.text.x, graphicsOps[i].u.text.y);
      tft.setTextColor(ST7735_BLACK);
      tft.print(graphicsOps[i].u.text.text);
      free(graphicsOps[i].u.text.text), graphicsOps[i].u.text.text = NULL;
      break;

    case GOPS_RECT:
      tft.drawRect(graphicsOps[i].u.rect.x, graphicsOps[i].u.rect.y, graphicsOps[i].u.rect.w, graphicsOps[i].u.rect.h, ST7735_BLACK);
      break;
 
    case GOPS_LINE:
      tft.drawLine(graphicsOps[i].u.line.x1, graphicsOps[i].u.line.y1, graphicsOps[i].u.line.x2, graphicsOps[i].u.line.y2, ST7735_BLACK);
      break;
    }
  }
  graphicsOpsCount = 0;
}

void
myPrint(int x, int y, const char *text, int colour)
{
  tft.setTextColor(colour);
  tft.setCursor(x, y);
  tft.print(text);
  graphicsOps[graphicsOpsCount].type = GOPS_TEXT;
  graphicsOps[graphicsOpsCount].u.text.x = x;
  graphicsOps[graphicsOpsCount].u.text.y = y;
  graphicsOps[graphicsOpsCount].u.text.text = strdup(text);
  graphicsOpsCount++;  
}

#define BLOCKSIZE 16
#define BLOCKHEIGHT 32
#define CHARRADIUS 10
#define LEFTGAP 3

int
playBlockGame(const char *level)
{
  int len = strlen(level);
  int deltaSmallX = 4;
  int curLevel = 1;
  int oldCharY = 0, charY = 0;
  bool inJump = false;
  int jumpPosition = 0, jumpLevel = 0;
  int score = 0;

  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(5, 10);
  tft.setTextColor(ST7735_WHITE);
  tft.printf("Player Ready\n");
  tft.printf("  Swipe to Jump\n");
  delay(3000);
  tft.fillScreen(ST7735_BLACK);
 
  for (int x = 1; level[x + (128 / BLOCKSIZE)]; x++) {
    if ((x % 16) == 0) {
      if (deltaSmallX < (BLOCKSIZE / 2)) {
        deltaSmallX *= 2;
      }
      curLevel++;
    }
    score++;

    for (int smallx = 0; smallx < BLOCKSIZE; smallx += deltaSmallX) {
      char text[100];
      
      myClearScreen();
      snprintf(text, sizeof(text), "Level %i\nScore %i\n", curLevel, score);
      myPrint(0, 0, text, ST7735_WHITE);

      if (inJump) {
        if (jumpPosition >= 128) {
          if (jumpLevel < (BLOCKSIZE)) {
            jumpLevel += deltaSmallX;
          } else {
            jumpPosition += 2 * deltaSmallX;         
          }
          if (jumpPosition >= 256) {
            inJump = false;
            jumpPosition = 0;
            jumpLevel = 0;
          }
        } else {
          jumpPosition += 2 * deltaSmallX;
        }
      }
      if (((smallx / deltaSmallX) % 3) == 0 && jumpPosition == 0 && mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
        inJump = true;
        jumpPosition = 0;
      }

      charY = 140 - CHARRADIUS - (2 * BLOCKHEIGHT + 32) * 0.5 * (1 + sineTable[(jumpPosition + 192) % 256]);
      myDrawRect(BLOCKSIZE * LEFTGAP, charY, CHARRADIUS, CHARRADIUS, ST7735_YELLOW);   
      
      for (int screenx = -BLOCKSIZE; screenx < (128 + BLOCKSIZE) && (x + screenx / BLOCKSIZE) < len; screenx += BLOCKSIZE) {
        ESP.wdtFeed();
        switch (level[x + screenx / BLOCKSIZE]) {
        case '.':
          myDrawLine(BLOCKSIZE - smallx + screenx, 140, BLOCKSIZE - smallx + screenx + BLOCKSIZE, 140, ST7735_WHITE);
          break;
          
        case 'x':
          myDrawLine(BLOCKSIZE - smallx + screenx, 140, BLOCKSIZE - smallx + screenx, 140 - BLOCKHEIGHT, ST7735_WHITE);
          myDrawLine(BLOCKSIZE - smallx + BLOCKSIZE + screenx, 140, BLOCKSIZE - smallx + BLOCKSIZE + screenx, 140 - BLOCKHEIGHT, ST7735_WHITE);
          myDrawLine(BLOCKSIZE - smallx + screenx, 140 - BLOCKHEIGHT, BLOCKSIZE - smallx + BLOCKSIZE + screenx, 140 - BLOCKHEIGHT, ST7735_WHITE);
          break;
          
        case 'X':
          myDrawLine(BLOCKSIZE - smallx + screenx, 140, BLOCKSIZE - smallx + screenx, 140 - 2*BLOCKHEIGHT, ST7735_WHITE);
          myDrawLine(BLOCKSIZE - smallx + BLOCKSIZE + screenx, 140, BLOCKSIZE - smallx + BLOCKSIZE + screenx, 140 - 2*BLOCKHEIGHT, ST7735_WHITE);
          myDrawLine(BLOCKSIZE - smallx + screenx, 140 - 2*BLOCKHEIGHT, BLOCKSIZE - smallx + BLOCKSIZE + screenx, 140 - 2*BLOCKHEIGHT, ST7735_WHITE);
          break;
        }
      }

      delay(50);
      
      if ((x + LEFTGAP) < len) {
        switch (level[x + LEFTGAP]) {
        case 'X':
          if (smallx >= (BLOCKSIZE - CHARRADIUS) && charY >= (140 - 2 * BLOCKHEIGHT - CHARRADIUS)) {
            tft.printf("\nDEAD!\nSwipe to jump!\n");
            delay(3000);
            return 0;
          }
          break;
        
        case 'x':
          if (smallx >= (BLOCKSIZE - CHARRADIUS) && charY >= (140 - BLOCKHEIGHT - CHARRADIUS)) {
            tft.printf("\nDEAD!\nSwipe to jump!\n");
            delay(3000);
            return 0;
          }
          break;
        }
      }
      if ((x + LEFTGAP - 1) < len) {
        switch (level[x + LEFTGAP - 1]) {
        case 'X':
          if (smallx <= 2*CHARRADIUS && charY >= (140 - 2 * BLOCKHEIGHT - CHARRADIUS)) {
            tft.printf("\nDEAD!\nSwipe to jump!\n");
            delay(3000);
            return 0;
          }
          break;
        
        case 'x':
          if (smallx <= 2*CHARRADIUS && charY >= (140 - BLOCKHEIGHT - CHARRADIUS)) {
            tft.printf("\nDEAD!\nSwipe to jump!\n");
            delay(3000);
            return 0;
          }
          break;
        }
      }
    }
  }
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(5, 10);
  tft.printf("YOU WON\n");
  xorprint(CTF_CARD_STR, CTF_CARD_KEY);
  tft.printf("\nLevel %i\nScore %i\n", curLevel, score);
  delay(6000);
  return 1;
}


void
drawYScrollerLine(int n, int offsetX, int offsetY, const char *text, int start, int len)
{
  int i;

  for (i = 0; i < n; i++) {
    char c;

    c = text[(start + i) % len];
    tft.setCursor(offsetX,
      (int)(((sineTable[offsetY % 256]+1.0)*15.0+5)));
    if (c == '\r' || c == '\n')
      tft.print(' ');
    else
      tft.print(c);
    offsetX += 12;
    offsetY += 2;
  }
}

void
drawYScroller(const char *text, int *offsetX, int *offsetY, int *index, int len)
{    
  tft.setTextSize(2);
  tft.setTextColor(ST7735_BLACK);
  drawYScrollerLine(9, *offsetX, *offsetY, text, *index, len);
  *offsetX = *offsetX - 2;
  if (*offsetX <= 0) {
    *offsetX = charWidth - *offsetX;
    *index = (*index + 1) % len;
  }
  *offsetY = *offsetY + 1;
  tft.setTextColor(ST7735_GREEN);
  drawYScrollerLine(9, *offsetX, *offsetY, text, *index, len);
  tft.setTextSize(1);
}

void
drawBobCircle(const char *text, int colour, int offset)
{
  ESP.wdtFeed();
  myClearScreen();
  for (unsigned int i = 0, angle = 0; angle < 256, text[i]; angle += 16, i++) {
    unsigned int x, y;

    if (isalpha(text[i])) {
      unsigned char *ch;

      ch = characterSet[tolower(text[i]) - 'a'];
      for (int r = 7; r >= 0; r--) {
        for (int c = 0; c < 8; c++) {
          if (ch[r*8 + c]) {
            x = 64 + (1.0 + (float)r/16.0) * 40.0 * cosineTable[(angle + offset + 2*c) % 256];
            y = 80 - (1.0 + (float)r/16.0) * 50.0 * sineTable[(angle + offset + 2*c) % 256];
            myDrawCirc(x, y, 1, colour);     
          }
        }
      }
    }
  }
}


void
drawBobs(const char *staticText, int n)
{
  for (int count = 0; count < n; count++) {
    for (int angle = 0; angle < 256; angle++) {
      drawBobCircle(staticText, ST7735_YELLOW, angle);
    }
  }
}

#define TWEETLISTSIZE 2

typedef char char200[200];
int tweetListHead = 0;
int numTweets = 0;
char200 tweetListBuf[TWEETLISTSIZE];

void
printTweets()
{
  char200 twitterBuf;

  if (WiFi.status() != WL_CONNECTED && numTweets == 0)
    return;

  tft.fillScreen(ST7735_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ST7735_YELLOW);
  tft.setCursor(0, 0);
  tft.printf("Live twitter ...\n");

  if (WiFi.status() == WL_CONNECTED && getNetworkMsg(twitterBuf, sizeof(twitterBuf), "/BSidesTwitterStream.txt") != 0) {
    for (int i = 0; i < numTweets; i++) {
      if (strcmp(tweetListBuf[(i + tweetListHead) % TWEETLISTSIZE], twitterBuf) == 0) {
        goto out;
      }
    }
    if (numTweets < TWEETLISTSIZE) {
      strncpy(tweetListBuf[numTweets], twitterBuf, sizeof(char200));
      numTweets++;
    } else {
      strncpy(tweetListBuf[(tweetListHead + TWEETLISTSIZE) % TWEETLISTSIZE], twitterBuf, sizeof(char200));
      tweetListHead = (tweetListHead + 1) % TWEETLISTSIZE;
    }
  }
out:
  for (int i = 0; i < numTweets; i++) {
    if (i & 1) {
      tft.setTextColor(ST7735_WHITE);
    } else {
      tft.setTextColor(ST7735_GREEN);
    }
    tft.printf("%s", tweetListBuf[(i + tweetListHead) % TWEETLISTSIZE]);
  }

  for (int i = 0; i < 600; i++) {
    if (checkCard())
      return;
    delay(10);
  }
}


void testroundrects() {
  tft.fillScreen(ST7735_BLACK);
  int color = 100;
  int i;
  int t;
  for(t = 0 ; t <= 2; t+=1) {
    int x = 0;
    int y = 35;
    int w = tft.width()-2;
    int h = tft.height()-35;
    for(i = 0 ; i <= 16; i+=1) {
      tft.drawRoundRect(x, y, w, h, 5, color);
      x+=2;
      y+=3;
      w-=4;
      h-=6;
      color+=1100;
    }
    color+=100;
  }
}

void
drawCredits()
{
  tft.fillScreen(ST7735_BLACK);
  testroundrects();
  for (int i = 0; i < 200; i++) {
    tft.setCursor(0,0);
    tft.setTextSize(2);
    tft.setTextColor(staticTextColor += 300);
    tft.println("   Badge\n  Credits");

    tft.setTextColor(ST7735_WHITE);
    tft.setTextSize(1);

    tft.println();
    tft.println();
    tft.println();
    
    tft.setTextColor(ST7735_WHITE);    
    tft.printf("   Hardware:\n");
    tft.setTextColor(ST7735_GREEN);    
    tft.printf("    Peter Filmore\n");
    tft.setTextColor(ST7735_WHITE);    
    tft.printf("   Software:\n");
    tft.setTextColor(ST7735_GREEN);    
    tft.printf("    Silvio\n");
    tft.setTextColor(ST7735_WHITE);    
    tft.printf("   Everything else:\n");
    tft.setTextColor(ST7735_GREEN);    
    tft.printf("    Kylie\n");
    tft.setTextColor(ST7735_WHITE);    
    tft.printf("   with ");
    tft.setTextColor(ST7735_GREEN);    
    tft.printf("Paul Harvey\n");

    if (checkCard())
      return;

    delay(10);
  }

}
void loop() {
#if 1
  char twitterBuf[200], msgBuf[300];
  int screenLen = 3;
  
  connectWifi();
  tft.fillScreen(ST7735_BLACK);
  tft.invertDisplay(true);
  delay(200);
  tft.invertDisplay(false);
  drawBobCircle("BSidesCbr", ST7735_YELLOW, 120);
  drawPulsingCircleText(1, "      Department of Defiance");
  tft.invertDisplay(true);
  delay(200);
  tft.invertDisplay(false);
  freeAll();

  printTweets();

  if (playBlockGame(blockGame)) { 
  }

  tft.fillScreen(ST7735_BLACK);
  tft.invertDisplay(true);
  delay(200);
  tft.invertDisplay(false);
  init3d();
  initWireframeBox(0, 0, 0, 3, 3, 3);
  drawRotating3d("hi there", 2);
  freeAll();

  tft.invertDisplay(true);
  delay(200);
  tft.invertDisplay(false);
  init3d();
  initWireframePyramid(0, 0, 0, 3, 3, 3);
  drawRotating3d("hi there", 2);
  freeAll();  

  tft.invertDisplay(true);
  delay(200);
  tft.invertDisplay(false);
  init3d();
  initWireframePyramid(-5, -3, 2, 1, 2, 1);
  initWireframePyramid(-5,  4, 3, 0.5, 1, 1);
  initWireframeBox( 2,  1, 1, 2, 1, 1);
  initWireframeBox( 5, -6, 0, 1, 1, 2.5);
  drawRotating3d("hi there", 2);
  freeAll();

  drawCredits();
  
#endif

return;
  // put your main code here, to run repeatedly:
  boolean rc = false;

return;

  if (++delayCount != 100)
    return;

  delayCount = 0;
  
  Serial.printf("Checking WiFi\n");
  
  // Set WiFi to station mode and disconnect from an AP if it was previously connected

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

#if 0
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(0,0);
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.print("Scanning..");
#endif

  int nets;
  
  nets = WiFi.scanNetworks(); // scan for access points

  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(0,0);
  tft.setTextColor(ST7735_YELLOW);
  tft.print("Found:");
  tft.println(nets);

  tft.setTextColor(ST7735_WHITE);
  rc = false;
  
  for (int s=0; s<nets; s++) {
    if (rc == true) {
      rc = false;
      tft.setTextColor(ST7735_WHITE);
    } else {
      rc = true;
      tft.setTextColor(ST7735_CYAN);
    }
    
    tft.print(WiFi.SSID(s));
    tft.print(" ");
    tft.print(WiFi.RSSI(s));
    tft.println("dBm ");

    /*uint8_t  * bssid = WiFi.BSSID(s);
    
    tft.print(bssid[0], HEX);
    tft.print(bssid[1], HEX);
    tft.print(bssid[2], HEX);
    tft.print(bssid[3], HEX);
    tft.print(bssid[4], HEX);
    tft.println(bssid[5], HEX);*/
  }
}

void graphV() {
  double gscale = (1023 * 0.00615) / 160; // gives scaling factor for X axis
  int vx;
  
  //tft.fillRect(0,0,159,10, ST7735_BLACK);
  tft.drawFastHLine(0,0,159,ST7735_BLACK);
  tft.drawFastHLine(0,1,159,ST7735_BLACK);
  tft.drawFastHLine(0,2,159,ST7735_BLACK);
  tft.drawFastHLine(0,3,159,ST7735_BLACK);
  tft.drawFastHLine(0,4,159,ST7735_BLACK);

  //tft.fillRect(0,0,vx,10, ST7735_GREEN);
}
unsigned char characterSet[26][64] = {
/* A */ {
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 1, 1, 1, 1, 0, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 1, 1, 1, 1, 1, 1, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
	},
/* B */ {
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 1, 1, 1, 1, 1, 0, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 1, 1, 1, 1, 1, 0, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 1, 1, 1, 1, 1, 0, 0, 
	},
/* C */ {
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 1, 1, 1, 1, 1, 0, 
		0, 1, 1, 0, 0, 0, 0, 0, 
		0, 1, 1, 0, 0, 0, 0, 0, 
		0, 1, 1, 0, 0, 0, 0, 0, 
		0, 0, 1, 1, 1, 1, 1, 0, 
	},
/* D */ {
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 1, 1, 1, 1, 1, 0, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 1, 1, 1, 1, 1, 0, 0, 
	},
/* E */ {
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 1, 1, 1, 1, 1, 1, 0, 
		0, 1, 1, 0, 0, 0, 0, 0, 
		0, 1, 1, 1, 1, 0, 0, 0, 
		0, 1, 1, 0, 0, 0, 0, 0, 
		0, 1, 1, 1, 1, 1, 1, 0, 
	},
/* F */ {
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 1, 1, 1, 1, 1, 1, 0, 
		0, 1, 1, 0, 0, 0, 0, 0, 
		0, 1, 1, 1, 1, 0, 0, 0, 
		0, 1, 1, 0, 0, 0, 0, 0, 
		0, 1, 1, 0, 0, 0, 0, 0, 
	},
/* G */ {
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 1, 1, 1, 1, 1, 0, 
		0, 1, 1, 0, 0, 0, 0, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 0, 1, 1, 1, 1, 1, 0, 
	},
/* H */ {
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 1, 1, 1, 1, 1, 1, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
	},
/* I */ {
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 1, 1, 1, 1, 0, 0, 
		0, 0, 0, 1, 1, 0, 0, 0, 
		0, 0, 0, 1, 1, 0, 0, 0, 
		0, 0, 0, 1, 1, 0, 0, 0, 
		0, 0, 1, 1, 1, 1, 0, 0, 
	},
/* J */ {
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 1, 1, 0, 
		0, 0, 0, 0, 0, 1, 1, 0, 
		0, 0, 0, 0, 0, 1, 1, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 0, 1, 1, 1, 1, 0, 0, 
	},
/* K */ {
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 1, 1, 0, 1, 1, 0, 0, 
		0, 1, 1, 1, 1, 0, 0, 0, 
		0, 1, 1, 0, 1, 1, 0, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
	},
/* L */ {
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 1, 1, 0, 0, 0, 0, 0, 
		0, 1, 1, 0, 0, 0, 0, 0, 
		0, 1, 1, 0, 0, 0, 0, 0, 
		0, 1, 1, 0, 0, 0, 0, 0, 
		0, 1, 1, 1, 1, 1, 1, 0, 
	},
/* M */ {
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 1, 1, 0, 0, 0, 1, 1, 
		0, 1, 1, 1, 1, 1, 1, 1, 
		0, 1, 1, 0, 1, 0, 1, 1, 
		0, 1, 1, 0, 0, 0, 1, 1, 
		0, 1, 1, 0, 0, 0, 1, 1, 
	},
/* N */ {
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 1, 1, 0, 0, 0, 1, 1, 
		0, 1, 1, 1, 1, 0, 1, 1, 
		0, 1, 1, 0, 1, 1, 1, 1, 
		0, 1, 1, 0, 0, 1, 1, 1, 
		0, 1, 1, 0, 0, 0, 1, 1, 
	},
/* O */ {
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 1, 1, 1, 1, 0, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 0, 1, 1, 1, 1, 0, 0, 
	},
/* P */ {
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 1, 1, 1, 1, 1, 0, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 1, 1, 1, 1, 1, 0, 0, 
		0, 1, 1, 0, 0, 0, 0, 0, 
		0, 1, 1, 0, 0, 0, 0, 0, 
	},
/* Q */ {
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 1, 1, 1, 1, 0, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 1, 1, 0, 1, 1, 1, 0, 
		0, 0, 1, 1, 1, 1, 0, 0, 
	},
/* R */ {
		0, 0, 0, 0, 0, 1, 1, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 1, 1, 1, 1, 1, 0, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 1, 1, 1, 1, 1, 0, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
	},
/* S */ {
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 1, 1, 1, 1, 1, 0, 
		0, 1, 1, 0, 0, 0, 0, 0, 
		0, 0, 1, 1, 1, 1, 0, 0, 
		0, 0, 0, 0, 0, 1, 1, 0, 
		0, 1, 1, 1, 1, 1, 0, 0, 
	},
/* T */ {
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 1, 1, 1, 1, 1, 1, 0, 
		0, 0, 0, 1, 1, 0, 0, 0, 
		0, 0, 0, 1, 1, 0, 0, 0, 
		0, 0, 0, 1, 1, 0, 0, 0, 
		0, 0, 0, 1, 1, 0, 0, 0, 
	},
/* U */ {
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 0, 1, 1, 1, 1, 0, 0, 
	},
/* V */ {
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 0, 1, 1, 1, 1, 0, 0, 
		0, 0, 0, 1, 1, 0, 0, 0, 
	},
/* W */ {
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 1, 1, 0, 0, 0, 1, 1, 
		0, 1, 1, 0, 0, 0, 1, 1, 
		0, 1, 1, 0, 1, 0, 1, 1, 
		0, 1, 1, 1, 1, 1, 1, 1, 
		0, 1, 1, 0, 0, 0, 1, 1, 
	},
/* X */ {
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 0, 1, 1, 1, 1, 0, 0, 
		0, 0, 0, 1, 1, 0, 0, 0, 
		0, 0, 1, 1, 1, 1, 0, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
	},
/* Y */ {
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 1, 1, 0, 0, 1, 1, 0, 
		0, 0, 1, 1, 1, 1, 0, 0, 
		0, 0, 0, 1, 1, 0, 0, 0, 
		0, 0, 0, 1, 1, 0, 0, 0, 
	},
/* Z */ {
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 0, 0, 0, 
		0, 1, 1, 1, 1, 1, 1, 0, 
		0, 0, 0, 0, 0, 1, 1, 0, 
		0, 0, 0, 1, 1, 0, 0, 0, 
		0, 1, 1, 0, 0, 0, 0, 0, 
		0, 1, 1, 1, 1, 1, 1, 0, 
  }
};
