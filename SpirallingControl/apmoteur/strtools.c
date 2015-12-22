#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include "gtksig.h"
#include <gtk/gtk.h>
#include "strtools.h"
#include "canfestival.h"

char* strtools_concat(char* s1, char* s2, ...) {
    va_list ap;
    va_start(ap,s2);
    int lg = strlen(s1)+strlen(s2)+1;
    char *result = malloc(lg * sizeof(char));
    strcpy(result, s1);
    strcat(result, s2);
    char *arg = va_arg(ap,char*);
    while ( arg != NULL) {
        lg += strlen(arg);
        char* tmp=result;
        result = malloc(lg * sizeof(char));
        strcpy(result, tmp);
        strcat(result, arg);
        arg = va_arg(ap,char*);
    }
    va_end(ap);
    return result;
}

char** strtools_split(char* chaine,const char* delim,int vide){

    char** tab=NULL;                    //tableau de chaine, tableau resultat
    char *ptr;                     //pointeur sur une partie de
    int sizeStr;                   //taille de la chaine à recupérer
    int sizeTab=0;                 //taille du tableau de chaine
    char* largestring;             //chaine à traiter

    int sizeDelim=strlen(delim);   //taille du delimiteur

    largestring = chaine;          //comme ca on ne modifie pas le pointeur d'origine
                                   //(faut ke je verifie si c bien nécessaire)


    while( (ptr=strstr(largestring, delim))!=NULL ){
           sizeStr=ptr-largestring;

           //si la chaine trouvé n'est pas vide ou si on accepte les chaine vide
           if(vide==1 || sizeStr!=0){
               //on alloue une case en plus au tableau de chaines
               sizeTab++;
               tab= (char**) realloc(tab,sizeof(char*)*sizeTab);

               //on alloue la chaine du tableau
               tab[sizeTab-1]=(char*) malloc( sizeof(char)*(sizeStr+1) );
               strncpy(tab[sizeTab-1],largestring,sizeStr);
               tab[sizeTab-1][sizeStr]='\0';
           }

           //on decale le pointeur largestring  pour continuer la boucle apres le premier elément traiter
           ptr=ptr+sizeDelim;
           largestring=ptr;
    }

    //si la chaine n'est pas vide, on recupere le dernier "morceau"
    if(strlen(largestring)!=0){
           sizeStr=strlen(largestring);
           sizeTab++;
           tab= (char**) realloc(tab,sizeof(char*)*sizeTab);
           tab[sizeTab-1]=(char*) malloc( sizeof(char)*(sizeStr+1) );
           strncpy(tab[sizeTab-1],largestring,sizeStr);
           tab[sizeTab-1][sizeStr]='\0';
    }
    else if(vide==1){ //si on fini sur un delimiteur et si on accepte les mots vides,on ajoute un mot vide
           sizeTab++;
           tab= (char**) realloc(tab,sizeof(char*)*sizeTab);
           tab[sizeTab-1]=(char*) malloc( sizeof(char)*1 );
           tab[sizeTab-1][0]='\0';

    }

    //on ajoute une case à null pour finir le tableau
    sizeTab++;
    tab= (char**) realloc(tab,sizeof(char*)*sizeTab);
    tab[sizeTab-1]=NULL;

    return tab;
}
// Compte le nombre d'occurence d'un caractere dans un fichier
int strtools_num_char_in_file(FILE* file, const char* delim) {
    int c = 0;
    int i = 0;
    fseek(file, 0, SEEK_SET);
    do {
        c = fgetc(file);
        if ( c == (int)*delim) {
            i = i+1;
        }
    } while(c != EOF);
    fseek(file, 0, SEEK_SET);
    return i;

}

// Compte le nombre d'occurence d'une chaine de caractère dans une autre chaine de caractère
int strtools_num_char_in_str(char* strRef, char* str2find) {
    int c,d,nchar,Nfind = 0;
    int size = strlen(strRef);    // taille de la chaine
    int size2 = strlen(str2find);
    for (c = 0 ; c < size-size2 ; c++) {
        nchar = 0;
        for (d = 0; d < size2 ; d++) {
            if (c+d <= size && strRef[c+d] == str2find[d]) { // si le caractère n° c est bien le caractère
                nchar++;
            }
        }
        if (nchar == size2) {
            Nfind++;
        }
    }
    return Nfind;             // on renvoie le compteur
}



// Affiche un resultat printf
void strtools_print_data(void* data,UNS8 type) {
    if (data != NULL) {
        if (type != 0x09) {
            int* pdata = data;
            if (type == 0x01) {
                printf("\n\nVAL : %d\n",(int)*pdata);
            } else if (type == 0x02) {
                printf("\n\nVAL : %d\n",(INTEGER8)*pdata);
            } else if (type == 0x03) {
                printf("\n\nVAL : %d\n",(INTEGER16)*pdata);
            } else if (type == 0x04) {
                printf("\n\nVAL : %d\n",(INTEGER32)*pdata);
            } else if (type == 0x05) {
                printf("\n\nVAL : %#.2x\n",(UNS8)*pdata);
            } else if (type == 0x06) {
                printf("\n\nVAL : %#.4x\n",(UNS16)*pdata);
            } else if (type == 0x07) {
                printf("\n\nVAL : %#.8x\n",(UNS32)*pdata);
            } else {
                printf("\nERREUR: Le type n'a pas ete defini"); exit(1);
            }

        } else {
            char *data_vs = data;
            printf("\n\nVAL :%s\n",(char*)data_vs);
        }
    } else {
        printf("Pointeur Null -> Arret"); exit(1);
    }
}
// Fonction permettant de transformer n'importe quel type en string
// pour affichage dans application gtk
// param1 : pointeur vers variable
// param2 : type
gchar* strtools_gnum2str(void *data,UNS8 type) {
    int* pdata = data;
    if (type == 0x01) {
        if ((int)*pdata == 1) {
            return "True";
        } else if ((int)*pdata == 0) {
            return "False";
        } else {
            printf("num2str error: no boolean\n");exit(1);
        }
    } else if (type == 0x02) {
        return g_strdup_printf ("%d", (INTEGER8)*pdata);
    } else if (type == 0x03) {
        return g_strdup_printf ("%d", (INTEGER16)*pdata);
    } else if (type == 0x04) {
        return g_strdup_printf ("%d", (INTEGER32)*pdata);
    } else if (type == 0x05) {
        return g_strdup_printf ("%#.2X", (UNS8)*pdata);
    } else if (type == 0x06) {
        return g_strdup_printf ("%#.4X", (UNS16)*pdata);
    } else if (type == 0x07) {
        return g_strdup_printf ("%#.8X", (UNS32)*pdata);
    } else if (type == 0x50) {
        return g_strdup_printf ("%u", (UNS32)*pdata);
    } else {
        printf("num2str error : Type inconnu\n"); exit(1);
    }
}

// Transformation d'un hexadecimal en binaire
char* strtools_hex2bin(UNS16* data,UNS8 type) {
    if (data != NULL) {
        gchar* strdat = strtools_gnum2str(data,type);
        char* res="";
        int i;
        for (i=2;strdat[i]!='\0';i++) {
            if (strdat[i] == '0') {
                res = strtools_concat(res,"0000",NULL);
            } else if (strdat[i] == '1') {
                res = strtools_concat(res,"0001",NULL);
            } else if (strdat[i] == '2') {
                res = strtools_concat(res,"0010",NULL);
            } else if (strdat[i] == '3') {
                res = strtools_concat(res,"0011",NULL);
            } else if (strdat[i] == '4') {
                res = strtools_concat(res,"0100",NULL);
            } else if (strdat[i] == '5') {
                res = strtools_concat(res,"0101",NULL);
            } else if (strdat[i] == '6') {
                res = strtools_concat(res,"0110",NULL);
            } else if (strdat[i] == '7') {
                res = strtools_concat(res,"0111",NULL);
            } else if (strdat[i] == '8') {
                res = strtools_concat(res,"1000",NULL);
            } else if (strdat[i] == '9') {
                res = strtools_concat(res,"1001",NULL);
            } else if (strdat[i] == 'A') {
                res = strtools_concat(res,"1010",NULL);
            } else if (strdat[i] == 'B') {
                res = strtools_concat(res,"1011",NULL);
            } else if (strdat[i] == 'C') {
                res = strtools_concat(res,"1100",NULL);
            } else if (strdat[i] == 'D') {
                res = strtools_concat(res,"1101",NULL);
            } else if (strdat[i] == 'E') {
                res = strtools_concat(res,"1110",NULL);
            } else if (strdat[i] == 'F') {
                res = strtools_concat(res,"1111",NULL);
            } else {
                printf("Caractère non reconnu : %c",strdat[i]); exit(1);
            }
        }
        strtools_inv_str(res);
        return res;
    } else {
        return NULL;
    }
}

void strtools_inv_str(char* chaine) {
    int i,n;
    char lettre;

    n = strlen(chaine);

    for(i = 0;i <n/2;i++)
    {
        lettre = chaine[i];
        chaine[i] = chaine[n-i-1];
        chaine[n-i-1] = lettre;
    }
}

// Convert Visible string to string
char* strtools_pt2vs(void* data) {
    char* data_vs = data;
    return data_vs;
}


