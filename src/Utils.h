#ifndef __UTILS__
#define __UTILS__

String join(int *data, int n, char sep)
{

    char buffer[100];
    char item[10];

    int p = 0;

    for (int i = 0; i < n; i++)
    {
        if (data[i] >= 0 && data[i] < 256)
        {
            itoa(data[i], item, 10);
            if (i != 0 && p < 99)
            {
                buffer[p++] = sep;
                buffer[p] = 0;
            }
            if (p + strlen(item) < 99)
            {
                strcpy(&buffer[p], item);
                p += strlen(item);
            }
        }
    }

    return String(buffer);
}

int splitter(char *in, int *out, char sep, int maxin, int maxout)
{

    char buffer[10];

    int pin = 0;
    int pout = 0;
    int n = 0;
    while (pin < maxin)
    {
        pout = 0;
        while (pin < maxin && in[pin] != sep)
        {
            char c = in[pin];
            if (pout < 9 && c >= '0' && c <= '9')
            { // Only numbers!!!
                buffer[pout++] = c;
            }
            pin++;
        }
        buffer[pout] = 0;

        if (n < maxout)
        {
            out[n++] = atoi(buffer);
        }
        pin++;
    }
    return n;
}

void testSplitter()
{

    const char *input = "10, 11, 12, 13,14,15";
    int inputLen = strlen((char *)input);
    int out[10] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
    int nout = 10;

    int found = splitter((char *)input, out, ',', inputLen, nout);
    String s = join(out, found, ',');

    Serial.println("====================================");

    for (int i = 0; i < found; i++)
    {
        Serial.println(out[i]);
    }

    Serial.println("Joined " + s);

    Serial.println("====================================");
}

#endif