extern crate proc_macro;

use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, ItemFn, LitStr};

#[proc_macro_attribute]
pub fn eepy_app(attr: TokenStream, item: TokenStream) -> TokenStream {
    let mut name: Option<LitStr> = None;
    let mut version: Option<LitStr> = None;
    let args_parser = syn::meta::parser(|meta| {
        if meta.path.is_ident("name") {
            name = Some(meta.value()?.parse()?);
            Ok(())
        } else if meta.path.is_ident("version") {
            version = Some(meta.value()?.parse()?);
            Ok(())
        } else {
            Err(meta.error("unsupported property"))
        }
    });

    parse_macro_input!(attr with args_parser);

    let fn_item = parse_macro_input!(item as ItemFn);
    let fn_name = fn_item.sig.ident.clone();

    let name = if name.is_none() {
        quote! { env!("CARGO_PKG_NAME") }
    } else {
        quote! { #name }
    };
    let version = if version.is_none() {
        quote! { env!("CARGO_PKG_VERSION") }
    } else {
        quote! { #version }
    };

    let output = quote! {
        #[unsafe(link_section = ".header")]
        #[used]
        static HEADER: ::eepy_sys::header::ProgramSlotHeader = ::eepy_sys::header::ProgramSlotHeader::partial(
            #name,
            #version,
            __eepy_sys_entry,
        );

        extern "C" fn __eepy_sys_entry() {
            #fn_name()
        }

        #fn_item
    };

    output.into()
}