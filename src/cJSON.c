/*
  Copyright (c) 2009-2017 Dave Gamble and cJSON contributors
  Simplified for this project.
*/

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <limits.h>
#include <ctype.h>
#include <float.h>

#include "cJSON.h"

static const unsigned char *global_error_pointer = NULL;

const char *cJSON_GetErrorPtr(void) {
    return (const char*)global_error_pointer;
}

static void *(*internal_malloc)(size_t sz) = malloc;
static void (*internal_free)(void *ptr) = free;

void cJSON_InitHooks(cJSON_Hooks* hooks) {
    if (!hooks) {
        internal_malloc = malloc;
        internal_free = free;
        return;
    }
    internal_malloc = (hooks->malloc_fn) ? hooks->malloc_fn : malloc;
    internal_free = (hooks->free_fn) ? hooks->free_fn : free;
}

static cJSON *cJSON_New_Item(void) {
    cJSON* node = (cJSON*)internal_malloc(sizeof(cJSON));
    if (node) memset(node, 0, sizeof(cJSON));
    return node;
}

void cJSON_Delete(cJSON *c) {
    cJSON *next;
    while (c) {
        next = c->next;
        if (!(c->type & cJSON_IsReference) && c->child) cJSON_Delete(c->child);
        if (!(c->type & cJSON_IsReference) && c->valuestring) internal_free(c->valuestring);
        if (!(c->type & cJSON_StringIsConst) && c->string) internal_free(c->string);
        internal_free(c);
        c = next;
    }
}

static const unsigned char *parse_value(cJSON *item, const unsigned char *value);

static const unsigned char *parse_string(cJSON *item, const unsigned char *str) {
    const unsigned char *ptr = str + 1;
    const unsigned char *end_ptr = ptr;
    char *out;
    int len = 0;
    
    if (*str != '\"') { global_error_pointer = str; return NULL; }
    
    while (*end_ptr != '\"' && *end_ptr) {
        if (*end_ptr == '\\') end_ptr++;
        end_ptr++;
        len++;
    }
    
    out = (char*)internal_malloc(len + 1);
    if (!out) return NULL;
    
    item->valuestring = out;
    item->type = cJSON_String;
    
    while (ptr < end_ptr) {
        if (*ptr != '\\') *out++ = *ptr++;
        else {
            ptr++;
            switch (*ptr) {
                case 'b': *out++ = '\b'; break;
                case 'f': *out++ = '\f'; break;
                case 'n': *out++ = '\n'; break;
                case 'r': *out++ = '\r'; break;
                case 't': *out++ = '\t'; break;
                case '\"': *out++ = '\"'; break;
                case '\\': *out++ = '\\'; break;
                default: *out++ = *ptr; break;
            }
            ptr++;
        }
    }
    *out = 0;
    return end_ptr + 1;
}

static const unsigned char *parse_number(cJSON *item, const unsigned char *num) {
    double n = 0, sign = 1, scale = 0;
    int subscale = 0, signsubscale = 1;
    
    if (*num == '-') { sign = -1; num++; }
    if (*num == '0') num++;
    if (*num >= '1' && *num <= '9') {
        do { n = (n * 10.0) + (*num++ - '0'); } while (*num >= '0' && *num <= '9');
    }
    if (*num == '.' && num[1] >= '0' && num[1] <= '9') {
        num++;
        do { n = (n * 10.0) + (*num++ - '0'); scale--; } while (*num >= '0' && *num <= '9');
    }
    if (*num == 'e' || *num == 'E') {
        num++;
        if (*num == '+') num++; else if (*num == '-') { signsubscale = -1; num++; }
        while (*num >= '0' && *num <= '9') subscale = (subscale * 10) + (*num++ - '0');
    }
    
    n = sign * n * pow(10.0, (scale + subscale * signsubscale));
    item->valuedouble = n;
    item->valueint = (int)n;
    item->type = cJSON_Number;
    return num;
}

static const unsigned char *skip(const unsigned char *in) {
    while (in && *in && *in <= 32) in++;
    return in;
}

static const unsigned char *parse_array(cJSON *item, const unsigned char *value) {
    cJSON *child;
    if (*value != '[') { global_error_pointer = value; return NULL; }
    
    item->type = cJSON_Array;
    value = skip(value + 1);
    if (*value == ']') return value + 1;
    
    item->child = child = cJSON_New_Item();
    if (!item->child) return NULL;
    value = parse_value(child, value);
    if (!value) return NULL;
    value = skip(value);
    
    while (*value == ',') {
        cJSON *new_item = cJSON_New_Item();
        if (!new_item) return NULL;
        child->next = new_item;
        new_item->prev = child;
        child = new_item;
        value = skip(parse_value(child, skip(value + 1)));
        if (!value) return NULL;
    }
    
    if (*value == ']') return value + 1;
    global_error_pointer = value;
    return NULL;
}

static const unsigned char *parse_object(cJSON *item, const unsigned char *value) {
    cJSON *child;
    if (*value != '{') { global_error_pointer = value; return NULL; }
    
    item->type = cJSON_Object;
    value = skip(value + 1);
    if (*value == '}') return value + 1;
    
    item->child = child = cJSON_New_Item();
    if (!item->child) return NULL;
    
    value = skip(parse_string(child, value));
    if (!value) return NULL;
    child->string = child->valuestring;
    child->valuestring = NULL;
    
    if (*value != ':') { global_error_pointer = value; return NULL; }
    value = skip(parse_value(child, skip(value + 1)));
    if (!value) return NULL;
    
    while (*value == ',') {
        cJSON *new_item = cJSON_New_Item();
        if (!new_item) return NULL;
        child->next = new_item;
        new_item->prev = child;
        child = new_item;
        value = skip(value + 1);
        value = skip(parse_string(child, value));
        if (!value) return NULL;
        child->string = child->valuestring;
        child->valuestring = NULL;
        if (*value != ':') { global_error_pointer = value; return NULL; }
        value = skip(parse_value(child, skip(value + 1)));
        if (!value) return NULL;
    }
    
    if (*value == '}') return value + 1;
    global_error_pointer = value;
    return NULL;
}

static const unsigned char *parse_value(cJSON *item, const unsigned char *value) {
    if (!value) return NULL;
    if (!strncmp((const char*)value, "null", 4)) { item->type = cJSON_NULL; return value + 4; }
    if (!strncmp((const char*)value, "false", 5)) { item->type = cJSON_False; return value + 5; }
    if (!strncmp((const char*)value, "true", 4)) { item->type = cJSON_True; item->valueint = 1; return value + 4; }
    if (*value == '\"') return parse_string(item, value);
    if (*value == '-' || (*value >= '0' && *value <= '9')) return parse_number(item, value);
    if (*value == '[') return parse_array(item, value);
    if (*value == '{') return parse_object(item, value);
    
    global_error_pointer = value;
    return NULL;
}

cJSON *cJSON_Parse(const char *value) {
    return cJSON_ParseWithOpts(value, 0, 0);
}

cJSON *cJSON_ParseWithOpts(const char *value, const char **return_parse_end, cJSON_bool require_null_terminated) {
    const unsigned char *end = 0;
    cJSON *c = cJSON_New_Item();
    global_error_pointer = 0;
    if (!c) return 0;
    
    end = parse_value(c, skip((const unsigned char*)value));
    if (!end) { cJSON_Delete(c); return 0; }
    
    if (require_null_terminated) {
        end = skip(end);
        if (*end) { cJSON_Delete(c); global_error_pointer = end; return 0; }
    }
    if (return_parse_end) *return_parse_end = (const char*)end;
    return c;
}

/* Utils */
int cJSON_GetArraySize(const cJSON *array) {
    cJSON *c = array->child;
    int i = 0;
    while (c) { i++; c = c->next; }
    return i;
}

cJSON *cJSON_GetArrayItem(const cJSON *array, int index) {
    cJSON *c = array->child;
    while (c && index > 0) { index--; c = c->next; }
    return c;
}

cJSON *cJSON_GetObjectItem(const cJSON *const object, const char *const string) {
    cJSON *c = object ? object->child : 0;
    while (c && c->string && strcmp(c->string, string) != 0) c = c->next;
    return c;
}

cJSON_bool cJSON_IsNumber(const cJSON *const item) { return item && item->type == cJSON_Number; }
cJSON_bool cJSON_IsString(const cJSON *const item) { return item && item->type == cJSON_String; }
cJSON_bool cJSON_IsArray(const cJSON *const item) { return item && item->type == cJSON_Array; }
cJSON_bool cJSON_IsObject(const cJSON *const item) { return item && item->type == cJSON_Object; }

double cJSON_GetNumberValue(const cJSON *item) { return item ? item->valuedouble : NAN; }
char *cJSON_GetStringValue(const cJSON *item) { return item ? item->valuestring : NULL; }
